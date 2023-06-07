pushd ../pseudo
if !(test -f sue_donimous.ko) then make; fi
sudo ./sue_donimous_unload >& /dev/null
sudo ./sue_donimous_load
popd
if !(test -f dma-proxy-test) then make; fi
#DMATXDEV=sue_donimous_tx0 DMARXDEV=sue_donimous_rx0 ./dma-proxy-test 8 64 1 &
#DMATXDEV=sue_donimous_tx1 DMARXDEV=sue_donimous_rx1 ./dma-proxy-test 8 64 1 &
DMATXDEV=sue_donimous_tx1 DMARXDEV=sue_donimous_rx1 ./dma-proxy-test 1 1 1 &
sleep 1
make clean
pushd ../pseudo
sudo ./sue_donimous_unload
make clean
popd
