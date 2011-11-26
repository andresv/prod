#!/bin/bash

TINYOS_TOOLS_VER=1.2.4
TINYOS_TOOLS=tinyos-tools-${TINYOS_TOOLS_VER}

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/${TINYOS_TOOLS}/debian/usr
    PACKAGES_DIR=$(pwd)/../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
fi
: ${PREFIX:=$(pwd)/../local}


build()
{
    set -e
    (
	aclocal
	libtoolize --automake --force --copy
	automake --foreign --add-missing --copy
	autoconf
    	./configure --prefix=${PREFIX}
    	make
    	make install-strip
    )
}

package()
{
    echo Packaging ${TINYOS_TOOLS}
    cd ${TINYOS_TOOLS}
    find debian/usr/bin/ -type f \
	| xargs perl -i -pe 's#'${PREFIX}'#/usr#'
    mkdir -p debian/DEBIAN
    cat ../tinyos-tools.control \
	| sed 's/@version@/'${TINYOS_TOOLS_VER}-`date +%Y%m%d`'/' \
        | sed 's/@architecture@/'${ARCH_TYPE}'/' \
        > debian/DEBIAN/control
    dpkg-deb --build debian ${PACKAGES_DIR}/tinyos-tools-${TINYOS_TOOLS_VER}.deb
}

case $1 in
    build)
	build
	;;

    clean)
	git clean -d -f -x
	;;

    deb)
	build
	package
	;;

    *)
	build
	;;
esac
