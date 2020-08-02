#!/bin/bash

NDK=/home/ytkim/Android/android-ndk-r15c/
ARCH=x86_64
SYSROOT=$NDK/platforms/android-21/arch-x86_64/
TOOLCHAIN=$NDK/toolchains/x86_64-4.9/prebuilt/linux-x86_64
CROSS_PREFIX=$TOOLCHAIN/bin/x86_64-linux-android-

echo "SYSROOT= "$SYSROOT
echo "TOOLCHAIN= "$TOOLCHAIN
echo "CROSS_PREFIX= "$CROSS_PREFIX

# ARCH         =arm
# ARCH         =aarch64
# ARCH         =x86
# ARCH         =x86_64
# ARCH         =mips
# ARCH         =mips64
# SYSROOT   =$NDK/platforms/android-21/arch-arm
# SYSROOT   =$NDK/platforms/android-21/arch-arm64
# SYSROOT   =$NDK/platforms/android-21/arch-x86
# SYSROOT   =$NDK/platforms/android-21/arch-x86_64
# SYSROOT   =$NDK/platforms/android-21/arch-mips
# SYSROOT   =$NDK/platforms/android-21/arch-mips64
# TOOLCHAIN =$NDK/toolchains/arm-linux-android-4.9/prebuilt/linux-x86_64
# TOOLCHAIN =$NDK/toolchains/aarch64-linux-android-4.9/prebuilt/linux-x86_64
# TOOLCHAIN =$NDK/toolchains/x86-4.9/prebuilt/linux-x86_64
# TOOLCHAIN =$NDK/toolchains/x86_64-4.9/prebuilt/linux-x86_64
# TOOLCHAIN =$NDK/toolchains/mipsel-linux-android-4.9/prebuilt/linux-x86_64
# TOOLCHAIN =$NDK/toolchains/mips64el-linux-android-4.9/prebuilt/linux-x86_64
# CROSS_PREFIX =$TOOLCHAIN/bin/arm-linux-android-
# CROSS_PREFIX =$TOOLCHAIN/bin/aarch64-linux-android-
# CROSS_PREFIX =$TOOLCHAIN/bin/i686-linux-android-
# CROSS_PREFIX =$TOOLCHAIN/bin/x86_64-linux-android-

function build_one
{
./configure \
--prefix=$PREFIX \
--enable-shared \
--disable-static \
--disable-doc \
--disable-ffmpeg \
--disable-ffplay \
--disable-ffprobe \
--disable-doc \
--disable-symver \
--enable-protocol=concat \
--enable-protocol=file \
--enable-muxer=mp4 \
--enable-demuxer=mpegts \
--cross-prefix=$CROSS_PREFIX \
--target-os=linux \
--arch=$ARCH \
--enable-cross-compile \
--sysroot=$SYSROOT \
--extra-cflags="-Os -fpic $ADDI_CFLAGS" \
--extra-ldflags="$ADDI_LDFLAGS" \
$ADDITIONAL_CONFIGURE_FLAG
#make clean all
#make -j2
#make install
#cp config.h $PREFIX
}

# fix1
#ABI=armeabi-v7a
#ABI=arm64-v8a
#ABI=x86
ABI=x86_64

PREFIX=$(pwd)/android/$ABI

# fix2
#ADDI_CFLAGS="-marm"
#ADDI_CFLAGS="-marm -mcpu=cortex-a7"
build_one


#------------------------------------------
#--- configure option ---
#------------------------------------------
#--arch=arm \
#--arch=arm64 \


#------------------------------------------
#--- gcc options for CFLAGS with -marm ---
#------------------------------------------
# example : 
# -marm -march=armv7 -mtune=cortex-a7 -mcpu=cortex-a7
#    or
# -marm -march=armv7 -mcpu=cortex-a7
#
# for 64-bit : 
# -marm -march=armv8

#-march=name
#           This specifies the name of the target ARM architecture.  GCC uses this name to determine what kind of instructions it
#           can emit when generating assembly code.  This option can be used in conjunction with or instead of the -mcpu= option.
#           Permissible names are: armv2, armv2a, armv3, armv3m, armv4, armv4t, armv5, armv5e, armv5t, armv5te, armv6, armv6-m,
#           armv6j, armv6k, armv6kz, armv6s-m, armv6t2, armv6z, armv6zk, armv7, armv7-a, armv7-m, armv7-r, armv7e-m, armv7ve,
#           armv8-a, armv8-a+crc, armv8.1-a, armv8.1-a+crc, armv8-m.base, armv8-m.main, armv8-m.main+dsp, iwmmxt, iwmmxt2.

#-mtune=name
#           This option specifies the name of the target ARM processor for which GCC should tune the performance of the code.
#           For some ARM implementations better performance can be obtained by using this option.  Permissible names are: arm2,
#           arm250, arm3, arm6, arm60, arm600, arm610, arm620, arm7, arm7m, arm7d, arm7dm, arm7di, arm7dmi, arm70, arm700,
#           arm700i, arm710, arm710c, arm7100, arm720, arm7500, arm7500fe, arm7tdmi, arm7tdmi-s, arm710t, arm720t, arm740t,
#           strongarm, strongarm110, strongarm1100, strongarm1110, arm8, arm810, arm9, arm9e, arm920, arm920t, arm922t,
#           arm946e-s, arm966e-s, arm968e-s, arm926ej-s, arm940t, arm9tdmi, arm10tdmi, arm1020t, arm1026ej-s, arm10e, arm1020e,
#           arm1022e, arm1136j-s, arm1136jf-s, mpcore, mpcorenovfp, arm1156t2-s, arm1156t2f-s, arm1176jz-s, arm1176jzf-s,
#           generic-armv7-a, cortex-a5, cortex-a7, cortex-a8, cortex-a9, cortex-a12, cortex-a15, cortex-a17, cortex-a32,
#           cortex-a35, cortex-a53, cortex-a57, cortex-a72, cortex-a73, cortex-r4, cortex-r4f, cortex-r5, cortex-r7, cortex-r8,
#           cortex-m33, cortex-m23, cortex-m7, cortex-m4, cortex-m3, cortex-m1, cortex-m0, cortex-m0plus,
#           cortex-m1.small-multiply, cortex-m0.small-multiply, cortex-m0plus.small-multiply, exynos-m1, marvell-pj4, xscale,
#           iwmmxt, iwmmxt2, ep9312, fa526, fa626, fa606te, fa626te, fmp626, fa726te, xgene1.

#-mcpu=name
#           This specifies the name of the target ARM processor.  GCC uses this name to derive the name of the target ARM
#           architecture (as if specified by -march) and the ARM processor type for which to tune for performance (as if
#           specified by -mtune).  Where this option is used in conjunction with -march or -mtune, those options take precedence
#           over the appropriate part of this option.
#
#           Permissible names for this option are the same as those for -mtune.


