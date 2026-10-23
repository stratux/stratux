/*
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	bootpartition_test.go: verifies the SD card write protection actually holds: the "boot partition"
	is really read-only (writes fail with EROFS) outside the rw window, writable only inside it, and
	read-only again afterwards, also when the write fails or an upload stalls.

	The tests mount a scratch ramfs (stand-in for /boot/firmware; same "mount -o remount,ro" path as
	on the Pi) and an overlay, so they need mount privileges. No real root needed: run them in an
	unprivileged user + mount namespace. bootpartition.go only uses the standard library, so it can be
	tested on its own without the cgo dependencies of the rest of package main:

		go test -c -o /tmp/bp.test main/bootpartition.go main/bootpartition_test.go
		unshare -r -m --propagation private /tmp/bp.test -test.v

	Without mount privileges the tests are skipped.
*/

package main

import (
	"bytes"
	"errors"
	"io"
	"mime/multipart"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"syscall"
	"testing"
	"time"
)

// scratchBootPartition mounts a ramfs, makes it read-only like stratux-pre-start.sh does, and points
// bootPartitionMountPoint at it for the duration of the test.
func scratchBootPartition(t *testing.T, readOnly bool) string {
	t.Helper()
	if os.Geteuid() != 0 {
		t.Skip("needs mount privileges, run inside: unshare -r -m --propagation private")
	}
	mp := t.TempDir()
	if out, err := exec.Command("mount", "-t", "ramfs", "stratux-test", mp).CombinedOutput(); err != nil {
		t.Skipf("cannot mount scratch ramfs: %s %s", err, out)
	}
	t.Cleanup(func() { exec.Command("umount", mp).Run() })
	if err := os.WriteFile(filepath.Join(mp, "stratux.conf"), []byte(`{"old":true}`), 0644); err != nil {
		t.Fatal(err)
	}
	if readOnly {
		if out, err := exec.Command("mount", "-o", "remount,ro", mp).CombinedOutput(); err != nil {
			t.Fatalf("remount ro: %s %s", err, out)
		}
	}
	old := bootPartitionMountPoint
	bootPartitionMountPoint = mp
	t.Cleanup(func() { bootPartitionMountPoint = old })
	t.Setenv("TMPDIR", t.TempDir()) // upload staging area, off the scratch partition
	return mp
}

// assertReadOnly checks that the kernel really refuses writes, not just that /proc/mounts says "ro".
func assertReadOnly(t *testing.T, mp string) {
	t.Helper()
	if !isMountReadOnly(mp) {
		t.Fatalf("%s is mounted rw, expected ro", mp)
	}
	err := os.WriteFile(filepath.Join(mp, "probe"), []byte("x"), 0644)
	if !errors.Is(err, syscall.EROFS) {
		t.Fatalf("write to %s: got %v, expected EROFS", mp, err)
	}
}

func assertWritable(t *testing.T, mp string) {
	t.Helper()
	if isMountReadOnly(mp) {
		t.Fatalf("%s is mounted ro, expected rw", mp)
	}
	probe := filepath.Join(mp, "probe")
	if err := os.WriteFile(probe, []byte("x"), 0644); err != nil {
		t.Fatalf("write to %s inside rw window: %v", mp, err)
	}
	os.Remove(probe)
}

func readFile(t *testing.T, path string) string {
	t.Helper()
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	return string(b)
}

func TestBootPartitionReadOnlyOutsideWindow(t *testing.T) {
	mp := scratchBootPartition(t, true)
	conf := filepath.Join(mp, "stratux.conf")
	assertReadOnly(t, mp)

	err := withBootPartitionWritable(func() error {
		assertWritable(t, mp)
		return writeFileAtomic(conf, []byte(`{"new":true}`), 0644)
	})
	if err != nil {
		t.Fatal(err)
	}
	assertReadOnly(t, mp)
	if got := readFile(t, conf); got != `{"new":true}` {
		t.Fatalf("stratux.conf = %q", got)
	}
	if _, err := os.Stat(conf + ".tmp"); !os.IsNotExist(err) {
		t.Fatalf("temp file left behind: %v", err)
	}
}

func TestBootPartitionReadOnlyAgainAfterFailedWrite(t *testing.T) {
	mp := scratchBootPartition(t, true)
	boom := errors.New("boom")
	if err := withBootPartitionWritable(func() error { return boom }); err != boom {
		t.Fatalf("got %v, expected the error of fn", err)
	}
	assertReadOnly(t, mp)
}

func TestBootPartitionReadOnlyAgainAfterPanic(t *testing.T) {
	mp := scratchBootPartition(t, true)
	func() {
		defer func() { recover() }()
		withBootPartitionWritable(func() error { panic("boom") })
	}()
	// net/http recovers handler panics, so stratuxrun keeps running: the card must be closed again
	// and the mutex released.
	assertReadOnly(t, mp)
	if !bootPartitionMutex.TryLock() {
		t.Fatal("bootPartitionMutex still locked after panic")
	}
	bootPartitionMutex.Unlock()
}

func TestBootPartitionAlreadyWritableIsLeftAlone(t *testing.T) {
	// Opt-out (/boot/firmware/.stratux-boot-rw) and dev machines: never remount behind the user's back.
	mp := scratchBootPartition(t, false)
	if err := withBootPartitionWritable(func() error { return nil }); err != nil {
		t.Fatal(err)
	}
	assertWritable(t, mp)
}

type failingReader struct{ n int }

func (r *failingReader) Read(p []byte) (int, error) {
	if r.n <= 0 {
		return 0, errors.New("connection reset")
	}
	n := copy(p, bytes.Repeat([]byte("x"), r.n))
	r.n -= n
	return n, nil
}

func TestAtomicWriteKeepsOldFileOnFailure(t *testing.T) {
	mp := scratchBootPartition(t, true)
	conf := filepath.Join(mp, "stratux.conf")
	err := withBootPartitionWritable(func() error {
		return writeFileAtomicFrom(conf, &failingReader{n: 10}, 0644)
	})
	if err == nil {
		t.Fatal("expected an error")
	}
	assertReadOnly(t, mp)
	if got := readFile(t, conf); got != `{"old":true}` {
		t.Fatalf("stratux.conf damaged by failed write: %q", got)
	}
	if _, err := os.Stat(conf + ".tmp"); !os.IsNotExist(err) {
		t.Fatalf("temp file left behind: %v", err)
	}
}

func TestAtomicWriteOverwritesStaleTempFile(t *testing.T) {
	mp := scratchBootPartition(t, false)
	conf := filepath.Join(mp, "stratux.conf")
	os.WriteFile(conf+".tmp", []byte("leftover from a power cut, much longer than the new content"), 0644)
	if err := writeFileAtomic(conf, []byte("new"), 0644); err != nil {
		t.Fatal(err)
	}
	if got := readFile(t, conf); got != "new" {
		t.Fatalf("stratux.conf = %q", got)
	}
	if _, err := os.Stat(conf + ".tmp"); !os.IsNotExist(err) {
		t.Fatalf("temp file left behind: %v", err)
	}
}

// A stalled update upload must not hold the boot partition rw, nor block saveSettings().
func TestStalledUploadKeepsBootPartitionReadOnly(t *testing.T) {
	mp := scratchBootPartition(t, true)
	updates := filepath.Join(mp, "StratuxUpdates")

	pr, pw := io.Pipe()
	mw := multipart.NewWriter(pw)
	reader := multipart.NewReader(pr, mw.Boundary())

	type result struct {
		path string
		err  error
	}
	done := make(chan result, 1)
	go func() {
		p, err := receiveUpdateFile(reader, updates)
		done <- result{p, err}
	}()

	fw, err := mw.CreateFormFile("update_file", "../../update-stratux-v9.9.sh")
	if err != nil {
		t.Fatal(err)
	}
	payload := bytes.Repeat([]byte("#!/bin/sh\n"), 100000) // ~1 MB
	if _, err := fw.Write(payload[:len(payload)/2]); err != nil {
		t.Fatal(err)
	}

	// The client now stalls mid-transfer.
	time.Sleep(200 * time.Millisecond)
	assertReadOnly(t, mp)
	if !bootPartitionMutex.TryLock() {
		t.Fatal("bootPartitionMutex is held while the upload body is still streaming")
	}
	bootPartitionMutex.Unlock()
	conf := filepath.Join(mp, "stratux.conf")
	saved := make(chan error, 1)
	go func() {
		saved <- withBootPartitionWritable(func() error { return writeFileAtomic(conf, []byte("saved"), 0644) })
	}()
	select {
	case err := <-saved:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("settings save blocked by a stalled upload")
	}
	assertReadOnly(t, mp)

	// The client resumes and finishes.
	go func() {
		fw.Write(payload[len(payload)/2:])
		mw.Close()
		pw.Close()
	}()
	var res result
	select {
	case res = <-done:
	case <-time.After(10 * time.Second):
		t.Fatal("upload did not finish")
	}
	if res.err != nil {
		t.Fatal(res.err)
	}
	want := filepath.Join(updates, "update-stratux-v9.9.sh") // no path traversal out of StratuxUpdates
	if res.path != want {
		t.Fatalf("stored as %q, expected %q", res.path, want)
	}
	if got := readFile(t, want); got != string(payload) {
		t.Fatalf("uploaded file corrupted (%d bytes, expected %d)", len(got), len(payload))
	}
	assertReadOnly(t, mp)
	entries, _ := os.ReadDir(updates)
	if len(entries) != 1 {
		t.Fatalf("expected only the update in %s, got %v", updates, entries)
	}
	staged, _ := os.ReadDir(os.Getenv("TMPDIR"))
	if len(staged) != 0 {
		t.Fatalf("staged upload not cleaned up from TMPDIR: %v", staged)
	}
}

func TestAbortedUploadLeavesNothingBehind(t *testing.T) {
	mp := scratchBootPartition(t, true)
	updates := filepath.Join(mp, "StratuxUpdates")
	pr, pw := io.Pipe()
	mw := multipart.NewWriter(pw)
	reader := multipart.NewReader(pr, mw.Boundary())
	done := make(chan error, 1)
	go func() {
		_, err := receiveUpdateFile(reader, updates)
		done <- err
	}()
	fw, _ := mw.CreateFormFile("update_file", "stratux-v9.9-arm64.deb")
	fw.Write([]byte("partial"))
	pw.CloseWithError(errors.New("client went away"))
	if err := <-done; err == nil {
		t.Fatal("expected an error for an aborted upload")
	}
	assertReadOnly(t, mp)
	if _, err := os.Stat(updates); !os.IsNotExist(err) {
		t.Fatalf("aborted upload touched the boot partition: %v", err)
	}
	staged, _ := os.ReadDir(os.Getenv("TMPDIR"))
	if len(staged) != 0 {
		t.Fatalf("staged upload not cleaned up from TMPDIR: %v", staged)
	}
}

// The "SD Card: Protected" badge must reflect what is mounted, not whether /overlay/robase exists.
func TestOverlayDetectionUsesMountedFSType(t *testing.T) {
	mp := scratchBootPartition(t, false)
	if fstype, _ := mountInfo(mp); fstype != "ramfs" {
		t.Fatalf("mountInfo(%s) fstype = %q, expected ramfs", mp, fstype)
	}

	// Stack an overlay on a directory: mountInfo must report the topmost mount.
	base := t.TempDir()
	for _, d := range []string{"lower", "upper", "work", "merged"} {
		os.Mkdir(filepath.Join(base, d), 0755)
	}
	merged := filepath.Join(base, "merged")
	opts := "lowerdir=" + filepath.Join(base, "lower") + ",upperdir=" + filepath.Join(base, "upper") + ",workdir=" + filepath.Join(base, "work")
	if out, err := exec.Command("mount", "-t", "overlay", "overlay", "-o", opts, merged).CombinedOutput(); err != nil {
		t.Skipf("cannot mount overlay: %s %s", err, out)
	}
	defer exec.Command("umount", merged).Run()
	if fstype, _ := mountInfo(merged); fstype != "overlay" {
		t.Fatalf("mountInfo(%s) fstype = %q, expected overlay", merged, fstype)
	}

	// And for / it must agree with the check stratux-pre-start.sh uses.
	out, err := exec.Command("findmnt", "-n", "-o", "FSTYPE", "/").Output()
	if err != nil {
		t.Skipf("findmnt: %s", err)
	}
	want := strings.TrimSpace(string(out)) == "overlay"
	if got := isOverlayRootActive(); got != want {
		t.Fatalf("isOverlayRootActive() = %v, findmnt says / is %q", got, strings.TrimSpace(string(out)))
	}
}
