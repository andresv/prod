#!/bin/bash

UISP_VER=20050519tinyos
UISP=uisp-${UISP_VER}

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/${UISP}/debian/usr
    PACKAGES_DIR=$(pwd)/../../../../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
fi
: ${PREFIX:=$(pwd)/../../../../local}


build()
{
    set -e
    (
	./bootstrap
    	./configure --prefix=${PREFIX}
    	make
    	make install-strip
    )
}

package()
{
    echo Packaging ${UISP}
    cd ${UISP}
    find debian/usr/bin/ -type f \
	| xargs perl -i -pe 's#'${PREFIX}'#/usr#'
    mkdir -p debian/DEBIAN
    cat ../uisp.control \
	| sed 's/@version@/'${UISP_VER}-`date +%Y%m%d`'/' \
        | sed 's/@architecture@/'${ARCH_TYPE}'/' \
        > debian/DEBIAN/control
    dpkg-deb --build debian ${PACKAGES_DIR}/uisp-${UISP_VER}.deb
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
