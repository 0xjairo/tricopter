Tricopter
========


Tricopter project based on the Maple Mini by LeafLabs
More info on the Maple Mini: http://leaflabs.com/devices/

https://github.com/jyros/tricopter


Installation
------------

Install the compiler toolchain as described in 
<http://leaflabs.com/docs/unix-toolchain.html> and 
summarized below.  Basically you grab the tar file from
the leaflabs.com website and add the bin/ directory to
your system PATH:

    $ wget http://static.leaflabs.com/pub/codesourcery/gcc-arm-none-eabi-latest-linux32.tar.gz
    $ tar xvzf gcc-arm-none-eabi-latest-linux32.tar.gz
    $ export PATH=$PATH:~/libmaple/arm/bin # or wherever these tools ended up

After you clone the project, you need to initialize and update the
libmape submodule:
    
    $ git submodule init
    $ git submodule update
    
This will fetch the libmaple module.  Next make the project

    $ make
    
Make sure you target maple_mini board.      
