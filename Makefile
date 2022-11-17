all: checkmakefiles
	cd src && $(MAKE)

clean: checkmakefiles
	cd src && $(MAKE) clean

cleanall: checkmakefiles
	cd src && $(MAKE) MODE=release clean
	cd src && $(MAKE) MODE=debug clean
	rm -f src/Makefile

makefiles:
	cd src && opp_makemake --make-so -f --deep -O out -KINET_PROJ=$(INET_ROOT) -KOS3_PROJ=$(HOME)/omnetprojects/os3 -DINET_IMPORT '-I$(OS3_PROJ)/src' '-I$(INET_PROJ)/src' -I/usr/include/x86_64-linux-gnu/curl '-L$(INET_PROJ)/src' '-L$(OS3_PROJ)/src' '-lINET$(D)' '-los3$(D)'

checkmakefiles:
	@if [ ! -f src/Makefile ]; then \
	echo; \
	echo '======================================================================='; \
	echo 'src/Makefile does not exist. Please use "make makefiles" to generate it!'; \
	echo '======================================================================='; \
	echo; \
	exit 1; \
	fi
