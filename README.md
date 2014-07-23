# Surface 3 Kernel

Ubuntu Trusty kernel with patches for the Microsoft Surface 3

Upstream: git://kernel.ubuntu.com/ubuntu/ubuntu-trusty.git.

## Binary packages

http://surface3.rbel.co/kernel

## Patches

Type Cover 3 Support:

Patch from: http://ubuntuforums.org/showthread.php?t=2231207&page=2&p=13070900#post13070900

## Building

make -j `getconf _NPROCESSORS_ONLN` deb-pkg LOCALVERSION=-surface3

See https://wiki.ubuntu.com/KernelTeam/GitKernelBuild
