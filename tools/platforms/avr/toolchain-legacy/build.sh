#!/bin/bash

BINUTILS_VER=2.17
BINUTILS=binutils-${BINUTILS_VER}
GCC_VER=4.1.2
GCC=gcc-${GCC_VER}

AVRLIBC_VER=1.4.7
AVRLIBC=avr-libc-${AVRLIBC_VER}

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/debian/usr
    PACKAGES_DIR=$(pwd)/../../../../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
    mkdir -p ${PACKAGES_DIR/${ARCH_TYPE}/all}
fi
: ${PREFIX:=$(pwd)/../../../../local}

download()
{
    [[ -a ${BINUTILS}a.tar.bz2 ]] \
	|| wget http://ftp.gnu.org/gnu/binutils/${BINUTILS}a.tar.bz2
    [[ -a ${GCC}.tar.bz2 ]] \
	|| wget http://ftp.gnu.org/gnu/gcc/gcc-${GCC_VER}/${GCC}.tar.bz2
    [[ -a ${AVRLIBC}.tar.bz2 ]] \
	|| wget http://download.savannah.gnu.org/releases/avr-libc/${AVRLIBC}.tar.bz2
}

build_binutils()
{
    echo Unpacking ${BINUTILS}a.tar.bz2
    rm -rf ${BINUTILS}
    tar -xjf ${BINUTILS}a.tar.bz2
    set -e
    (
	cd ${BINUTILS}
	patch -p0 < ../avr-binutils.patch
	./configure \
	    --prefix=${PREFIX} \
	    --mandir=${PREFIX}/share/man \
	    --target=avr \
	    --program-prefix=avr- \
	    --disable-nls \
	    --disable-werror
	make
	make install
	rm -rf ${PREFIX}{/info,/lib/libiberty.a,/share/locale}
	find ${PREFIX} -empty | xargs rm -rf
    )
    ( cd $PREFIX ; find . -type f ) > avr-binutils.files
}

package_binutils()
{
    set -e
    (
	VER=${BINUTILS_VER}
	cd ${BINUTILS}
	mkdir -p debian/DEBIAN
	cat ../avr-binutils.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	rsync -a ../debian/usr debian
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/avr-binutils-tinyos-legacy-${VER}.deb
    )
}

build_gcc()
{
    echo Unpacking ${GCC}.tar.gz
    rm -rf ${GCC}
    tar -xjf ${GCC}.tar.bz2
    
    set -e
    (
	PATH=${PREFIX}/bin:${PATH}
	cd ${GCC}
	patch -p0 < ../avr-gcc.patch
	./configure \
	    --prefix=${PREFIX} \
	    --target=avr \
	    --program-prefix=avr- \
	    --disable-nls \
	    --enable-languages=c \
	    --disable-libssp
	make
	make install
	rm -rf ${PREFIX}{/info,/lib/libiberty.a,/share/locale}
	( cd ${PREFIX} ; find man | grep -v avr | xargs rm -rf )
	find ${PREFIX} -empty | xargs rm -rf
    )
    ( cd $PREFIX ; find . -type f ) > avr-gcc.files
}

package_gcc()
{
    set -e
    (
	VER=${GCC_VER}
	cd ${GCC}
	mkdir -p debian/DEBIAN
	cat ../avr-gcc.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	rsync -a ../debian/usr debian
	(
	    cd debian/usr
	    cat ../../../avr-binutils.files | xargs rm -rf
	    until find . -empty -exec rm -rf {} \; &> /dev/null
	    do : ; done
	)
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/avr-gcc-tinyos-legacy-${VER}.deb
    )
}

build_libc()
{
    echo Unpacking ${AVRLIBC}.tar.gz
    rm -rf ${AVRLIBC}
    tar -xjf ${AVRLIBC}.tar.bz2

    set -e
    (
	PATH=${PREFIX}/bin:${PATH}
	cd ${AVRLIBC}

	(
	    cd include/avr
	    patch -p0 < ../../../sfr_defs.patch
	)

	./configure \
	    --prefix=${PREFIX} \
	    --program-prefix=avr- \
	    --build=$(./config.guess) \
	    --host=avr
	make
	make install

	(
	    cd include/avr
	    patch -R -p0 < ../../../sfr_defs.patch
	)
    )
}

package_libc()
{
    set -e
    (
	VER=${AVRLIBC_VER}
	cd ${AVRLIBC}
	rsync -a -m ../debian/usr debian
	find debian/usr/bin/ -type f \
	    | xargs perl -i -pe 's#'${PREFIX}'#/usr#'
	(
	    cd debian/usr
	    cat ../../../avr-gcc.files | xargs rm -rf
	    until find . -empty -exec rm -rf {} \; &> /dev/null
	    do : ; done
	)
	mkdir -p debian/DEBIAN
	cat ../avr-libc.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/avr-libc-tinyos-legacy-${VER}.deb
    )
}

package_dummy()
{
    set -e
    (
	mkdir -p tinyos
	cd tinyos
	mkdir -p debian/DEBIAN
	cat ../avr-tinyos.control \
	    | sed 's/@version@/'$(date +%Y%m%d)'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR/${ARCH_TYPE}/all}/avr-tinyos-legacy.deb
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
	remove ${BINUTILS} ${GCC} ${AVRLIBC} tinyos *.files debian
	;;

    veryclean)
	remove {${BINUTILS},${GCC},${AVRLIBC}}{,.tar.gz,.tar.bz2,a.tar.bz2}
	remove tinyos *.files debian
	;;

    deb)
	download
	build_binutils
	package_binutils
	build_gcc
	package_gcc
	build_libc
	package_libc
	package_dummy
	;;

    *)
	download
	build_binutils
	build_gcc
	build_libc
	;;
esac
