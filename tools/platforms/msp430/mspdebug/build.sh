#!/bin/bash

MSPDEBUG_VER=0.16
MSPDEBUG=mspdebug-${MSPDEBUG_VER}

ARCH_TYPE=$(dpkg-architecture -qDEB_HOST_ARCH)
if [[ "$1" == deb ]]
then
    PREFIX=$(pwd)/debian/usr
    PACKAGES_DIR=$(pwd)/../../../../packages/${ARCH_TYPE}
    mkdir -p ${PACKAGES_DIR}
fi
: ${PREFIX:=$(pwd)/../../../../local}

download()
{
    [[ -a ${MSPDEBUG}.tar.gz ]] \
	|| wget http://sourceforge.net/projects/mspdebug/files/${MSPDEBUG}.tar.gz
}

build_mspdebug()
{
    echo Unpacking ${MSPDEBUG}.tar.gz
    rm -rf ${MSPDEBUG}
    tar xzf ${MSPDEBUG}.tar.gz
    set -e
    (
	cd ${MSPDEBUG}
	make -j4
	make install PREFIX=${PREFIX}
    )
}

package_mspdebug()
{
    set -e
    (
	VER=${MSPDEBUG_VER}
	mkdir -p debian/DEBIAN
	cat mspdebug.control \
	    | sed 's/@version@/'${VER}-$(date +%Y%m%d)'/' \
	    | sed 's/@architecture@/'${ARCH_TYPE}'/' \
	    > debian/DEBIAN/control
	dpkg-deb --build debian \
	    ${PACKAGES_DIR}/mspdebug-${VER}.deb
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
	remove ${MSPDEBUG} debian
	;;

    veryclean)
	remove mspdebug-* debian
	;;

    deb)
	download
	build_mspdebug
	package_mspdebug
	;;

    *)
	download
	build_mspdebug
	;;
esac
