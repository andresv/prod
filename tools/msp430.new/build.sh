#!/bin/bash

BINUTILS_VER=2.21.1
BINUTILS=binutils-${BINUTILS_VER}
GCC_VER=4.5.3
GCC_CORE=gcc-core-${GCC_VER}
GCC=gcc-${GCC_VER}
MSPGCC_VER=20110716
MSPGCC=mspgcc-${MSPGCC_VER}

PATCHES="
  msp430-binutils-2.21.1-20110716-sf3143071.patch
  msp430-binutils-2.21.1-20110716-sf3379341.patch
  msp430-binutils-2.21.1-20110716-sf3386145.patch
  msp430-gcc-4.5.3-20110706-sf3370978.patch
  msp430-gcc-4.5.3-20110706-sf3390964.patch
  msp430-libc-20110612-sf3387164.patch
  msp430mcu-20110613-sf3379189.patch
  msp430mcu-20110613-sf3384550.patch"

MPFR_VER=2.4.2
MPFR=mpfr-${MPFR_VER}
GMP_VER=4.3.2
GMP=gmp-${GMP_VER}
MPC_VER=0.8.1
MPC=mpc-${MPC_VER}

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
    [[ -a ${GCC_CORE}.tar.gz ]] \
	|| wget http://ftp.gnu.org/gnu/gcc/${GCC}/${GCC_CORE}.tar.gz

    [[ -a ${MPFR}.tar.gz ]] \
	|| wget http://www.mpfr.org/${MPFR}/${MPFR}.tar.gz
    [[ -a ${GMP}.tar.gz ]] \
	|| wget http://ftp.gnu.org/gnu/gmp/${GMP}.tar.gz
    [[ -a ${MPC}.tar.gz ]] \
	|| wget http://www.multiprecision.org/mpc/download/${MPC}.tar.gz

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

    # Download some patches to the MSP430 LTS release
    for f in ${PATCHES}
    do
	[[ -a ${f} ]] \
	    || wget http://sourceforge.net/projects/mspgcc/files/Patches/LTS/20110716/${f}
    done
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
	echo Extra patches...
	cat ../msp430-binutils-*.patch | patch -p1
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

    echo Unpacking ${MPFR}.tar.gz
    rm -rf ${MPFR}
    tar -xzf ${MPFR}.tar.gz

    echo Unpacking ${GMP}.tar.gz
    rm -rf ${GMP}
    tar -xzf ${GMP}.tar.gz

    echo Unpacking ${MPC}.tar.gz
    rm -rf ${MPC}
    tar -xzf ${MPC}.tar.gz

    set -e
    (
    	cd $GCC
	ln -s ../${MPFR} mpfr
	ln -s ../${GMP} gmp
	ln -s ../${MPC} mpc
    	cat ../${MSPGCC}/msp430-gcc-${GCC_VER}-*.patch | patch -p1
	echo Extra patches...
	cat ../msp430-gcc-*.patch | patch -p1
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
	echo Extra patches...
	cat ../msp430mcu-*.patch | patch -p1
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
	cd ${MSP430LIBC}
	echo Extra patches...
	cat ../msp430-libc-*.patch | patch -p1
	cd src
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

package_dummy()
{
    set -e
    (
	mkdir -p tinyos
	cd tinyos
	mkdir -p debian/DEBIAN
	cat ../msp430-tinyos.control \
	    | sed 's/@version@/'$(date +%Y%m%d)'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR/${ARCH_TYPE}/all}/msp430-tinyos.deb
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
	remove ${BINUTILS} ${GCC} ${MSPGCC} ${MSP430MCU} ${MSP430LIBC} tinyos gcc *.files debian
	;;

    veryclean)
	remove {${BINUTILS},${GCC},${GCC_CORE}}{,.tar.gz} tinyos gcc *.files debian
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
	package_dummy
	;;

    *)
	download
	build_binutils
	build_mcu
	build_gcc
	build_libc
	;;
esac
