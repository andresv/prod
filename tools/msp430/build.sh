#!/bin/bash

BINUTILS_VER=2.17
BINUTILS=binutils-${BINUTILS_VER}
GCC_VER=3.2.3
GCC_CORE=gcc-core-${GCC_VER}
GCC=gcc-${GCC_VER}
MSPGCC_VER=20060801cvs
MSPGCC=mspgcc-cvs

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
    [[ -a mspgcc-cvs.tar.gz ]] \
	|| http://tinyos.stanford.edu/tinyos/toolchain/repo/mspgcc-cvs.tar.gz
}

build_binutils()
{
    echo Unpacking ${BINUTILS}.tar.gz
    rm -rf ${BINUTILS}
    tar -xzf ${BINUTILS}.tar.gz
    set -e
    (
	cd ${BINUTILS}
	perl -i.orig \
	    -pe 's/define (LEX_DOLLAR) 0/undef	$1/' \
	    gas/config/tc-msp430.h
	./configure \
	    --prefix=${PREFIX} \
	    --target=msp430 \
	    --program-prefix=msp430- \
	    --disable-werror
	make
	make install
	rm -rf ${PREFIX}{/info,/lib/libiberty.a,/share/locale}
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
    echo Unpacking ${MSPGCC}.tar.gz
    rm -rf ${MSPGCC}
    tar -xzf ${MSPGCC}.tar.gz
    echo Unpacking ${GCC_CORE}.tar.gz
    rm -rf ${GCC}
    tar -xzf ${GCC_CORE}.tar.gz
    
    set -e
    (
	PATH=${PREFIX}/bin:${PATH}
	cd ${GCC}
	cp -a ../${MSPGCC}/gcc/gcc-3.3/* .
	./configure \
	    --prefix=${PREFIX} \
	    --target=msp430 \
	    --program-prefix=msp430-
	CPPFLAGS=-D_FORTIFY_SOURCE=0 make
	make install
	rm -rf ${PREFIX}{/info,/lib/libiberty.a,/share/locale}
	( cd ${PREFIX} ; find man | grep -v msp430 | xargs rm -rf )
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

build_libc()
{
    set -e
    (
	PATH=${PREFIX}/bin:${PATH}
	cd ${MSPGCC}/msp430-libc/src

	(
	    cd ../include/msp430
	    patch -p0 < ../../../../adc12.patch
	)

	[[ -d msp1 ]] || mkdir msp1
	[[ -d msp2 ]] || mkdir msp2
	perl -i.orig \
	    -pe 's{^(prefix\s*=\s*)(.*)}{${1}'"$PREFIX"'}' \
	    Makefile
	make
	make install

	(
	    cd ../include/msp430
	    patch -R -p0 < ../../../../adc12.patch
	)
    )
}

package_libc()
{
    set -e
    (
	VER=${MSPGCC_VER}
	cd ${MSPGCC}
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
	remove ${BINUTILS} ${GCC} ${MSPGCC} *.files debian
	;;

    veryclean)
	remove {${BINUTILS},${GCC},${GCC_CORE}}{,.tar.gz} *.files debian
	remove ${MSPGCC}
	;;

    deb)
	download
	build_binutils
	package_binutils
	build_gcc
	package_gcc
	build_libc
	package_libc
	;;

    *)
	download
	build_binutils
	build_gcc
	build_libc
	;;
esac
