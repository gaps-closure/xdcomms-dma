SUBDIRS = api test_app pseudo

all:
	for dir in $(SUBDIRS); do \
	$(MAKE) -C $$dir; \
	done

