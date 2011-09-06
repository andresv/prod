#!/bin/bash

AVRDUDE_VER=5.4
AVRDUDE=avrdude-${AVRDUDE_VER}

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/${AVRDUDE}/debian/usr
    PACKAGES_DIR=$(pwd)/../../../../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
fi
: ${PREFIX:=$(pwd)/../../../../local}

download()
{
    [[ -a ${AVRDUDE}.tar.gz ]] \
	|| wget http://download.savannah.gnu.org/releases/avrdude/${AVRDUDE}.tar.gz
}

build_avrdude()
{
    echo Unpacking ${AVRDUDE}.tar.gz
    rm -rf ${AVRDUDE}
    tar -xzf ${AVRDUDE}.tar.gz
    set -e
    (
	cd ${AVRDUDE}
	sed -i -e 's|-DCONFIG_DIR=\\"$(sysconfdir)\\"|-DCONFIG_DIR=\\"/etc\\"|' Makefile.{am,in}
	patch -p1 < ../avrdude-5.4-mib510.patch
	./configure \
	    --prefix=${PREFIX} \
	    --sysconfdir=${PREFIX}/../etc \
	    --mandir=${PREFIX}/share/man
	make
	make install
    )
}

package_avrdude()
{
    set -e
    (
	VER=${AVRDUDE_VER}
	cd ${AVRDUDE}
	mkdir -p debian/DEBIAN
	cat ../avrdude.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/avrdude-tinyos-${VER}.deb
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
	remove ${AVRDUDE}
	;;

    veryclean)
	remove ${AVRDUDE}{,.tar.gz}
	;;

    deb)
	download
	build_avrdude
	package_avrdude
	;;

    *)
	download
	build_avrdude
	;;
esac
