#!/bin/bash
wget http://downloads.sourceforge.net/project/contiki/Contiki/Contiki%202.7/contiki-2.7.zip
unzip contiki-2.7.zip
rm contiki-2.7.zip
mv contiki-2.7 contiki

echo "Create link for Mulle platform into Contiki platforms"
ln -s `pwd`/platform contiki/platform/mulle
