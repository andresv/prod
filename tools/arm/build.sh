#!/bin/bash

CS_BASE=2011.03
CS_REV=42
CS_VERSION=${CS_BASE}-${CS_REV}
LOCAL_BASE=arm-${CS_VERSION}-arm-none-eabi
LOCAL_SOURCE=${LOCAL_BASE}.src.tar.bz2
SOURCE_URL=http://www.codesourcery.com/sgpp/lite/arm/portal/package8733/public/arm-none-eabi/${LOCAL_SOURCE}

TARGET=arm-none-eabi

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/debian/usr
    PACKAGES_DIR=$(pwd)/../../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
fi
: ${PREFIX:=$(pwd)/../../local}

BINUTILS=binutils-${CS_BASE}

download()
{
    [[ -a ${LOCAL_SOURCE} ]] || wget ${SOURCE_URL}
    [[ -d ${LOCAL_BASE} ]] || tar xjvf ${LOCAL_SOURCE}
}

build_binutils()
{
    echo Unpacking ${LOCAL_BASE}/binutils-*.tar.bz2
    rm -rf ${BINUTILS}
    tar -xjf ${LOCAL_BASE}/binutils-*.tar.bz2
    set -e
    (
    	cd ${BINUTILS}
    	./configure \
    	    --prefix=${PREFIX} \
	    --target=${TARGET} \
	    --program-prefix=${TARGET}- \
    	    --disable-nls \
    	    --disable-werror
    	make
    	make install
    	rm -rf ${PREFIX},/lib/libiberty.a,/share/info}
    	find ${PREFIX} -empty | xargs rm -rf
    )
    ( cd $PREFIX ; find . -type f ) > arm-binutils.files
}

package_binutils()
{
    set -e
    (
	VER=${CS_VERSION}
	cd ${BINUTILS}
	mkdir -p debian/DEBIAN
	cat ../arm-binutils.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	rsync -a ../debian/usr debian
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/arm-binutils-${VER}.deb
    )
}

build_gcc()
{
    echo Unpacking ${LOCAL_BASE}/gcc-*.tar.bz2
    rm -rf gcc-*-${CS_BASE}
    tar -xjf ${LOCAL_BASE}/gcc-*.tar.bz2
    
    echo Unpacking ${LOCAL_BASE}/mpfr-*.tar.bz2
    rm -rf mpfr-${CS_BASE}
    tar -xjf ${LOCAL_BASE}/mpfr-*.tar.bz2

    echo Unpacking ${LOCAL_BASE}/gmp-*.tar.bz2
    rm -rf gmp-${CS_BASE}
    tar -xjf ${LOCAL_BASE}/gmp-*.tar.bz2

    echo Unpacking ${LOCAL_BASE}/mpc-*.tar.bz2
    rm -rf mpc-?.?.?
    tar -xjf ${LOCAL_BASE}/mpc-*.tar.bz2

    set -e
    (
    	PATH=${PREFIX}/bin:${PATH}
    	cd gcc-*-${CS_BASE}
	ln -s ../mpfr-${CS_BASE} mpfr
	ln -s ../gmp-${CS_BASE} gmp
	ln -s ../mpc-?.?.? mpc
	mkdir build
	cd build
    	../configure \
    	    --prefix=${PREFIX} \
    	    --target=${TARGET} \
    	    --program-prefix=${TARGET}- \
    	    --disable-nls \
    	    --enable-languages=c \
    	    --disable-libssp \
            --with-newlib \
            --without-headers \
            --disable-shared \
            --disable-threads \
            --disable-libmudflap \
            --disable-libgomp \
            --disable-libstdcxx-pch \
            --disable-libunwind-exceptions \
            --disable-libffi  \
            --enable-extra-sgxxlite-multilibs
    	make -j4 all-gcc
    	make install-gcc
    	rm -rf ${PREFIX}{/lib/libiberty.a,/share/info}
    	( cd ${PREFIX} ; find share/man | grep -v arm | xargs rm -rf )
    	find ${PREFIX} -empty | xargs rm -rf
    )
    ( cd $PREFIX ; find . -type f ) > arm-gcc.files
}

package_gcc()
{
    set -e
    (
	VER=${CS_VERSION}
    	cd gcc-*-${CS_BASE}
	mkdir -p debian/DEBIAN
	cat ../arm-gcc.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	rsync -a ../debian/usr debian
	(
	    cd debian/usr
	    cat ../../../arm-binutils.files | xargs rm -rf
	    find . -empty | xargs rm -rf
	)
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/arm-gcc-${VER}.deb
    )
}

build_newlib()
{
    echo Unpacking ${LOCAL_BASE}/newlib-*.tar.bz2
    rm -rf newlib-${CS_BASE}
    tar -xjf ${LOCAL_BASE}/newlib-*.tar.bz2

    (
    	PATH=${PREFIX}/bin:${PATH}
	NEWLIB_FLAGS="-ffunction-sections -fdata-sections -DPREFER_SIZE_OVER_SPEED -D__OPTIMIZE_SIZE__ -Os -fomit-frame-pointer -fno-unroll-loops -D__BUFSIZ__=256 -mabi=aapcs"
    	cd newlib-${CS_BASE}
	mkdir build
	cd build
    	../configure \
    	    --prefix=${PREFIX} \
    	    --target=${TARGET} \
            --disable-newlib-supplied-syscalls \
            --disable-libgloss \
            --disable-nls \
            --disable-shared \
            --enable-newlib-io-long-long
	make -j4 CFLAGS_FOR_TARGET="${NEWLIB_FLAGS}" CCASFLAGS="${NEWLIB_FLAGS}"
	make install
    	rm -rf ${PREFIX}/share/info
    )
    ( cd $PREFIX ; find . -type f ) > arm-newlib.files
}

package_newlib()
{
    set -e
    (
	VER=${CS_VERSION}
	cd newlib-${CS_BASE}
	rsync -a -m ../debian/usr debian
	find debian/usr/bin/ -type f \
	    | xargs perl -i -pe 's#'${PREFIX}'#/usr#'
	(
	    cd debian/usr
	    cat ../../../arm-gcc.files | xargs rm -rf
	    until $(find . -empty)
	    do
		find . -empty | xargs rm -rf
	    done
	)
	mkdir -p debian/DEBIAN
	cat ../arm-newlib.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/arm-newlib-${VER}.deb
    )
}

build_gcc_again()
{
    echo Unpacking ${LOCAL_BASE}/gcc-*.tar.bz2
    rm -rf gcc-*-${CS_BASE}
    tar -xjf ${LOCAL_BASE}/gcc-*.tar.bz2

    echo Unpacking ${LOCAL_BASE}/mpfr-*.tar.bz2
    rm -rf mpfr-${CS_BASE}
    tar -xjf ${LOCAL_BASE}/mpfr-*.tar.bz2

    echo Unpacking ${LOCAL_BASE}/gmp-*.tar.bz2
    rm -rf gmp-${CS_BASE}
    tar -xjf ${LOCAL_BASE}/gmp-*.tar.bz2

    echo Unpacking ${LOCAL_BASE}/mpc-*.tar.bz2
    rm -rf mpc-?.?.?
    tar -xjf ${LOCAL_BASE}/mpc-*.tar.bz2
    
    cp -r ${PREFIX}/arm-none-eabi/include ${PREFIX}/arm-none-eabi/sys-include

    set -e
    (
    	PATH=${PREFIX}/bin:${PATH}
    	cd gcc-*-${CS_BASE}
	ln -s ../mpfr-${CS_BASE} mpfr
	ln -s ../gmp-${CS_BASE} gmp
	ln -s ../mpc-?.?.? mpc
	mkdir build
	cd build
    	../configure \
    	    --prefix=${PREFIX} \
    	    --target=${TARGET} \
    	    --program-prefix=${TARGET}- \
    	    --disable-nls \
    	    --enable-languages=c \
    	    --disable-libssp \
            --with-newlib \
            --without-headers \
            --disable-shared \
            --disable-threads \
            --disable-libmudflap \
            --disable-libgomp \
            --disable-libstdcxx-pch \
            --disable-libunwind-exceptions \
            --disable-libffi  \
            --enable-extra-sgxxlite-multilibs
    	make -j4
    	make install
    	rm -rf ${PREFIX}{/lib/libiberty.a,/share/info}
    	( cd ${PREFIX} ; find share/man | grep -v arm | xargs rm -rf )
    	find ${PREFIX} -empty | xargs rm -rf
    )
    ( cd $PREFIX ; find . -type f ) > arm-gcc-again.files
}

package_gcc_again()
{
    set -e
    (
	VER=${CS_VERSION}
    	cd gcc-*-${CS_BASE}
	mkdir -p debian/DEBIAN
	cat ../arm-gcc.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	rsync -a ../debian/usr debian
	(
	    cd debian/usr
	    cat ../../../arm-binutils.files | xargs rm -rf
	    diff ../../../arm-newlib.files ../../../arm-gcc.files \
		| grep '< ' | cut -d ' ' -f 2 | xargs rm -rf
	    until $(find . -empty)
	    do
		find . -empty | xargs rm -rf
	    done
	)
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/arm-gcc-${VER}.deb
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
	remove *-${CS_BASE} mpc-?.?.? *.files debian
	;;

    veryclean)
	remove *-${CS_BASE}{,.tar.bz2} mpc-?.?.? *.files debian
	;;

    deb)
	download
	build_binutils
	package_binutils
	build_gcc
	package_gcc
	build_newlib
	package_newlib
	build_gcc_again
	package_gcc_again
	;;

    *)
	download
	build_binutils
	build_gcc
	build_newlib
	build_gcc_again
	;;
esac
