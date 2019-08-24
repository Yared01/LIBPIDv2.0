obj =  libpid.o main.o
sdir = pid

main : $(obj)
	cc -o main $(obj) -lm -I pid
	
main.o : libpid.h

libpid.o : libpid.h

.PHONY: subdirs $(sdir)
subdirs: $(sdir)
$(SUBDIRS):
	$(MAKE) -C $@

clean :
	rm out1.dat $(obj)
