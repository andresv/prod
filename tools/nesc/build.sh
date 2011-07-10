#!/bin/bash

NESC_VER=1.3.3
NESC=nesc-${NESC_VER}

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/${NESC}/debian/usr
    PACKAGES_DIR=$(pwd)/../../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
fi
: ${PREFIX:=$(pwd)/../../local}

download()
{
    [[ -a ${NESC}.tar.gz ]] \
	|| wget http://downloads.sourceforge.net/project/nescc/nescc/v${NESC_VER}/${NESC}.tar.gz
}

build()
{
    echo Unpacking ${NESC}.tar.gz
    rm -rf ${NESC}
    tar -xzf ${NESC}.tar.gz
    set -e
    (
    	cd ${NESC}
    	./configure --prefix=${PREFIX}
    	make
    	make install-strip
    )
}

package()
{
    echo Packaging ${NESC}
    cd ${NESC}
    find debian/usr/bin/ -type f \
	| xargs perl -i -pe 's#'${PREFIX}'#/usr#'
    mkdir -p debian/DEBIAN
    cat ../nesc.control \
	| sed 's/@version@/'${NESC_VER}-`date +%Y%m%d`'/' \
        | sed 's/@architecture@/'${ARCH_TYPE}'/' \
        > debian/DEBIAN/control
    dpkg-deb --build debian ${PACKAGES_DIR}/nesc-${NESC_VER}.deb
}

remove()
{
    for f in $@
    do
	if [[ -a ${f} ]]
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

    build)
	build
	;;

    clean)
	remove ${NESC}
	;;

    veryclean)
	remove ${NESC}{,.tar.gz}
	;;

    deb)
	download
	build
	package
	;;

    *)
	download
	build
	;;
esac
