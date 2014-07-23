# Surface Pro 3 Kernel

Ubuntu Trusty kernel with patches for the Microsoft Surface 3

Upstream: git://kernel.ubuntu.com/ubuntu/ubuntu-trusty.git.

## Binary packages

http://surface3.rbel.co/kernel

## Patches

Type Cover 3 Support:

Patch from: http://ubuntuforums.org/showthread.php?t=2231207&page=2&p=13070900#post13070900

## Building

```
sudo apt-get install git build-essential kernel-package fakeroot libncurses5-dev

make -j `getconf _NPROCESSORS_ONLN` deb-pkg LOCALVERSION=-surface3
```

See https://wiki.ubuntu.com/KernelTeam/GitKernelBuild

## Touchpad support

Copy the `misc/xorg.conf` configuration to `/etc/X11/xorg.conf` and restart the session.

## Reading

Type Cover 2 bug:

https://bugzilla.kernel.org/show_bug.cgi?id=64811

Useful Reddit thread on the Surface Pro 1

http://www.reddit.com/r/SurfaceLinux/comments/2b1hf6/running_ubuntu_1404_on_surface_pro_1_full_time/

Ubuntu 14.04 on the Surface Pro 3 Reddit Thread:

http://www.reddit.com/r/SurfaceLinux/comments/2bhfk5/ubuntu_1404_on_the_surface_pro_3_thread/
