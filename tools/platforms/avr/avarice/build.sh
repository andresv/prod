#!/bin/bash

AVARICE_VER=2.11
AVARICE=avarice-${AVARICE_VER}

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/${AVARICE}/debian/usr
    PACKAGES_DIR=$(pwd)/../../../../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
fi
: ${PREFIX:=$(pwd)/../../../../local}

download()
{
    [[ -a ${AVARICE}.tar.bz2 ]] \
	|| wget http://sourceforge.net/projects/avarice/files/avarice/${AVARICE}/${AVARICE}.tar.bz2
}

build_avarice()
{
    echo Unpacking ${AVARICE}.tar.bz2
    rm -rf ${AVARICE}
    tar -xjf ${AVARICE}.tar.bz2
    set -e
    (
	cd ${AVARICE}
	./configure \
	    --prefix=${PREFIX}
	make
	make install
    )
}

package_avarice()
{
    set -e
    (
	VER=${AVARICE_VER}
	cd ${AVARICE}
	mkdir -p debian/DEBIAN
	cat ../avarice.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/avarice-tinyos-${VER}.deb
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
	remove ${AVARICE}
	;;

    veryclean)
	remove ${AVARICE}{,.tar.bz2}
	;;

    deb)
	download
	build_avarice
	package_avarice
	;;

    *)
	download
	build_avarice
	;;
esac
