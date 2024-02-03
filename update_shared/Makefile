BINDIR=/usr/local/bin

SCRIPT=update_shared.sh

install:
	@echo "Installing script to $(BINDIR)"
	@install -m 0755 $(SCRIPT) $(BINDIR)/$(SCRIPT)

uninstall:
	@echo "Removing script from $(BINDIR)"
	@rm -f $(BINDIR)/$(SCRIPT)