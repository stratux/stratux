/*
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	bootpartition.go: SD card write protection helpers.

	The root filesystem is protected by the overlay (see init-overlay / overlayctl) unless
	Persistent logging is on. The FAT boot partition (/boot/firmware) is not: it holds stratux.conf
	and is mounted rw by default. A FAT partition that is yanked from power while dirty is the most
	common way to corrupt a Stratux SD card. stratux-pre-start.sh therefore remounts it read-only,
	and every write we do to it goes through withBootPartitionWritable(), which opens a short rw
	window, syncs, and closes it again. Callers must prepare their data before opening the window
	(e.g. stage uploads in /tmp) so the window only lasts as long as the actual write.
*/

package main

import (
	"bufio"
	"bytes"
	"fmt"
	"io"
	"log"
	"mime/multipart"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"syscall"
)

// var, not const, so tests can point it at a scratch mount.
var bootPartitionMountPoint = "/boot/firmware"

var bootPartitionMutex sync.Mutex

// mountInfo returns the filesystem type and mount options of the given mount point, or "", nil if
// it is not mounted. When several mounts are stacked on the same point the last (topmost) one wins,
// like the kernel's view.
func mountInfo(mountPoint string) (fstype string, opts []string) {
	f, err := os.Open("/proc/mounts")
	if err != nil {
		return "", nil
	}
	defer f.Close()
	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		fields := strings.Fields(scanner.Text())
		if len(fields) >= 4 && fields[1] == mountPoint {
			fstype = fields[2]
			opts = strings.Split(fields[3], ",")
		}
	}
	return fstype, opts
}

func isMountReadOnly(mountPoint string) bool {
	_, opts := mountInfo(mountPoint)
	for _, o := range opts {
		if o == "ro" {
			return true
		}
	}
	return false
}

// isOverlayRootActive reports whether / is the tmpfs-backed overlay (i.e. root is write protected).
// /overlay/robase exists even when the overlay is disabled (Persistent logging), so check what is
// actually mounted on /, like overlay_is_active() in stratux-pre-start.sh.
func isOverlayRootActive() bool {
	fstype, _ := mountInfo("/")
	return fstype == "overlay"
}

func remountBootPartition(mode string) error {
	out, err := exec.Command("mount", "-o", "remount,"+mode, bootPartitionMountPoint).CombinedOutput()
	if err != nil {
		return fmt.Errorf("remount %s %s: %s (%s)", bootPartitionMountPoint, mode, err.Error(), strings.TrimSpace(string(out)))
	}
	return nil
}

// withBootPartitionWritable runs fn with the boot partition mounted rw. If the partition was read-only
// before, it is synced and remounted read-only afterwards, even if fn fails. If it is not mounted, or
// already rw (dev machines, or the user opted out via /boot/firmware/.stratux-boot-rw), fn is simply
// called. fn should only do local file I/O: anything slow (network reads) keeps the card writable and
// blocks every other caller, including saveSettings().
func withBootPartitionWritable(fn func() error) (err error) {
	bootPartitionMutex.Lock()
	defer bootPartitionMutex.Unlock()

	wasReadOnly := isMountReadOnly(bootPartitionMountPoint)
	if wasReadOnly {
		if err := remountBootPartition("rw"); err != nil {
			return err
		}
	}
	// Deferred so the partition is closed again even if fn panics (net/http recovers handler panics,
	// so stratuxrun would otherwise keep running with the card writable).
	defer func() {
		syscall.Sync()
		if wasReadOnly {
			if rerr := remountBootPartition("ro"); rerr != nil {
				log.Printf("SD card protection: %s\n", rerr.Error())
				if err == nil {
					err = rerr
				}
			}
		}
	}()
	return fn()
}

// writeFileAtomic writes data to a temp file next to path, fsyncs it and renames it over path.
// See writeFileAtomicFrom.
func writeFileAtomic(path string, data []byte, perm os.FileMode) error {
	return writeFileAtomicFrom(path, bytes.NewReader(data), perm)
}

// writeFileAtomicFrom copies r to path.tmp, fsyncs it and renames it over path, then fsyncs the
// directory. path is never truncated in place, so an interrupted write leaves the old file intact.
// On ext4 the rename itself is atomic. On FAT (/boot/firmware) it is not journaled, so this narrows
// the window in which a power cut can hurt to the rename itself, but cannot close it completely.
// The temp name is fixed, so a leftover from a crash is overwritten by the next write (and
// stratux-pre-start.sh removes a stale stratux.conf.tmp at boot).
func writeFileAtomicFrom(path string, r io.Reader, perm os.FileMode) error {
	tmp := path + ".tmp"
	fd, err := os.OpenFile(tmp, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, perm)
	if err != nil {
		return err
	}
	if _, err = io.Copy(fd, r); err == nil {
		err = fd.Sync()
	}
	if cerr := fd.Close(); err == nil {
		err = cerr
	}
	if err != nil {
		os.Remove(tmp)
		return err
	}
	if err = os.Rename(tmp, path); err != nil {
		os.Remove(tmp)
		return err
	}
	dir, err := os.Open(filepath.Dir(path))
	if err != nil {
		return err
	}
	err = dir.Sync()
	if cerr := dir.Close(); err == nil {
		err = cerr
	}
	if err != nil {
		return fmt.Errorf("fsync %s: %s", filepath.Dir(path), err.Error())
	}
	return nil
}

// receiveUpdateFile reads the "update_file" part of an update upload and stores it in baseDir.
// /boot/firmware is read-only during normal operation, so the upload is received into /tmp first:
// a slow or stalled transfer must not hold the boot partition rw, or the lock saveSettings() needs.
// Only the local copy onto the card runs inside the rw window. Returns the stored path, or "" if
// the form had no update_file.
func receiveUpdateFile(reader *multipart.Reader, baseDir string) (string, error) {
	var part *multipart.Part
	for {
		p, err := reader.NextPart()
		if err == io.EOF {
			return "", nil
		}
		if err != nil {
			return "", err
		}
		if p.FormName() == "update_file" {
			part = p
			break
		}
	}
	uploadFilename := filepath.Join(baseDir, filepath.Base(part.FileName()))

	staged, err := os.CreateTemp("", "stratux-update-*")
	if err != nil {
		return "", err
	}
	defer os.Remove(staged.Name())
	defer staged.Close()
	if _, err = io.Copy(staged, part); err != nil {
		return "", err
	}
	if _, err = staged.Seek(0, io.SeekStart); err != nil {
		return "", err
	}

	err = withBootPartitionWritable(func() error {
		// Ensure the upload directory exists. The SD card flow relies on the user
		// creating this path manually; the web flow has no such step, so create it
		// here to avoid a silent failure when os.OpenFile cannot create the file.
		if err := os.MkdirAll(baseDir, 0755); err != nil {
			return err
		}
		return writeFileAtomicFrom(uploadFilename, staged, 0666)
	})
	if err != nil {
		return "", err
	}
	return uploadFilename, nil
}
