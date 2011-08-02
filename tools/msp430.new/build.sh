#!/bin/bash

BINUTILS_VER=2.21.1
BINUTILS=binutils-${BINUTILS_VER}
GCC_VER=4.5.3
GCC_CORE=gcc-core-${GCC_VER}
GCC=gcc-${GCC_VER}
MSPGCC_VER=20110716
MSPGCC=mspgcc-${MSPGCC_VER}

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/debian/usr
    PACKAGES_DIR=$(pwd)/../../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
fi
: ${PREFIX:=$(pwd)/../../local}

download()
{
    [[ -a ${BINUTILS}.tar.gz ]] \
	|| wget http://ftp.gnu.org/gnu/binutils/${BINUTILS}.tar.gz
    [[ -a gcc-core-${GCC_VER}.tar.gz ]] \
	|| wget http://ftp.gnu.org/gnu/gcc/gcc-${GCC_VER}/gcc-core-${GCC_VER}.tar.gz
    [[ -a ${MSPGCC}.tar.bz2 ]] \
	|| wget http://sourceforge.net/projects/mspgcc/files/mspgcc/${MSPGCC}.tar.bz2
    # We need to unpack this in order to find what libc to download
    [[ -d ${MSPGCC} ]] \
        || tar xjf ${MSPGCC}.tar.bz2
    MSP430MCU_VER=$(cat ${MSPGCC}/msp430mcu.version)
    MSP430MCU=msp430mcu-${MSP430MCU_VER}
    [[ -a ${MSP430MCU}.tar.bz2 ]] \
	|| wget http://sourceforge.net/projects/mspgcc/files/msp430mcu/${MSP430MCU}.tar.bz2
    MSP430LIBC_VER=$(cat ${MSPGCC}/msp430-libc.version)
    MSP430LIBC=msp430-libc-${MSP430LIBC_VER}
    [[ -a ${MSP430LIBC}.tar.bz2 ]] \
	|| wget http://sourceforge.net/projects/mspgcc/files/msp430-libc/${MSP430LIBC}.tar.bz2
}

build_binutils()
{
    echo Unpacking ${BINUTILS}.tar.gz
    rm -rf ${BINUTILS}
    tar -xzf ${BINUTILS}.tar.gz
    set -e
    (
	cd ${BINUTILS}
	cat ../${MSPGCC}/msp430-binutils-${BINUTILS_VER}-*.patch | patch -p1
	../${BINUTILS}/configure \
	    --prefix=${PREFIX} \
	    --target=msp430
	make -j4
	make install
	rm -rf ${PREFIX}{/lib/libiberty.a,/share/info,/share/locale}
	find ${PREFIX} -empty | xargs rm -rf
    )
    ( cd $PREFIX ; find . -type f ) > msp430-binutils.files
}

package_binutils()
{
    set -e
    (
	VER=${BINUTILS_VER}
	cd ${BINUTILS}
	mkdir -p debian/DEBIAN
	cat ../msp430-binutils.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	rsync -a ../debian/usr debian
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/msp430-binutils-${VER}.deb
    )
}

build_gcc()
{
    echo Unpacking ${GCC_CORE}.tar.gz
    rm -rf ${GCC}
    tar -xzf ${GCC_CORE}.tar.gz
    set -e
    (
    	cd $GCC
    	cat ../${MSPGCC}/msp430-gcc-${GCC_VER}-*.patch | patch -p1
	mkdir build
	cd build
    	../configure \
    	    --prefix=${PREFIX} \
    	    --target=msp430 \
    	    --enable-languages=c
        CPPFLAGS=-D_FORTIFY_SOURCE=0 make -j4
    	make install
    	rm -rf ${PREFIX}{/lib/libiberty.a,/share/info,/share/locale,/share/man/man7}
    	find ${PREFIX} -empty | xargs rm -rf
    )
    ( cd $PREFIX ; find . -type f ) > msp430-gcc.files
}

package_gcc()
{
    set -e
    (
	VER=${GCC_VER}
	cd ${GCC}
	mkdir -p debian/DEBIAN
	cat ../msp430-gcc.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	rsync -a ../debian/usr debian
	(
	    cd debian/usr
	    cat ../../../msp430-binutils.files | xargs rm -rf
	    find . -empty | xargs rm -rf
	)
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/msp430-gcc-${VER}.deb
    )
}

build_mcu()
{
    echo Unpacking ${MSP430MCU}.tar.bz2
    rm -rf ${MSP430MCU}
    tar xjf ${MSP430MCU}.tar.bz2
    set -e
    (
	cd ${MSP430MCU}
	MSP430MCU_ROOT=$(pwd) scripts/install.sh ${PREFIX}
    )
    ( cd $PREFIX ; find . -type f ) > msp430mcu.files
}

package_mcu()
{
    set -e
    (
	VER=${MSP430MCU_VER}
	cd ${MSP430MCU}
	rsync -a -m ../debian/usr debian
	(
	    cd debian/usr
	    cat ../../../msp430-gcc.files | xargs rm -rf
	    until $(find . -empty)
	    do
		find . -empty | xargs rm -rf
	    done
	)
	mkdir -p debian/DEBIAN
	cat ../msp430mcu.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/msp430mcu-${VER}.deb
    )
}

build_libc()
{
    echo Unpacking ${MSP430LIBC}.tar.bz2
    rm -rf ${MSP430LIBC}
    tar xjf ${MSP430LIBC}.tar.bz2
    set -e
    (
	PATH=${PREFIX}/bin:${PATH}
	cd ${MSP430LIBC}/src
	make PREFIX=${PREFIX} -j4
	make PREFIX=${PREFIX} install
    )
}

package_libc()
{
    set -e
    (
	VER=${MSP430LIBC_VER}
	cd ${MSP430LIBC}
	rsync -a -m ../debian/usr debian
	(
	    cd debian/usr
	    cat ../../../msp430mcu.files | xargs rm -rf
	    until $(find . -empty)
	    do
		find . -empty | xargs rm -rf
	    done
	)
	mkdir -p debian/DEBIAN
	cat ../msp430-libc.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/msp430-libc-${VER}.deb
    )
}

remove()
{
    for f in $@
    do
	if [ -a ${f} ]
	then
	    echo Removing ${f}
    	    rm -rf $f
	fi
    done
}

case $1 in
    download)
	download
	;;

    clean)
	MSP430MCU_VER=$(cat ${MSPGCC}/msp430mcu.version)
	MSP430MCU=msp430mcu-${MSP430MCU_VER}
	MSP430LIBC_VER=$(cat ${MSPGCC}/msp430-libc.version)
	MSP430LIBC=msp430-libc-${MSP430LIBC_VER}
	remove ${BINUTILS} ${GCC} ${MSPGCC} ${MSP430MCU} ${MSP430LIBC} gcc *.files debian
	;;

    veryclean)
	remove {${BINUTILS},${GCC},${GCC_CORE}}{,.tar.gz} gcc *.files debian
	remove {${MSPGCC},msp430-libc-*,msp430mcu-*}{,.tar.bz2}
	;;

    deb)
	download
	build_binutils
	package_binutils
	build_gcc
	package_gcc
	build_mcu
        package_mcu
	build_libc
	package_libc
	;;

    *)
	download
	build_binutils
	build_mcu
	build_gcc
	build_libc
	;;
esac
