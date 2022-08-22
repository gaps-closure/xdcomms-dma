cd ../pseudo 
if !(test -f sue_donimous.ko)
then
  make;
fi
sudo ./sue_donimous_unload >& /dev/null
sudo ./sue_donimous_load
cd ../test
#DMATXDEV=sue_donimous_tx1 DMARXDEV=sue_donimous_rx0 ./dma-proxy-test 100 64 1
DMATXDEV=sue_donimous_tx0 DMARXDEV=sue_donimous_rx1 ./dma-proxy-test 16 64 1
sudo ../pseudo/sue_donimous_unload
