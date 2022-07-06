package common

import "os/user"

func StringInSlice(a string, list []string) bool {
	// TODO: When we are going to use go 1.18 we can use Slices.contains
    for _, b := range list {
        if b == a {
            return true
        }
    }
    return false
}

func IsRunningAsRoot() bool {
    usr, _ := user.Current()
    return usr.Username == "root"
}