pushd ../pseudo
if !(test -f sue_donimous.ko) then make; fi
sudo ./sue_donimous_unload >& /dev/null
sudo ./sue_donimous_load
popd
lsmod | grep sue_donimous
