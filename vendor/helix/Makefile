
ARCH:=$(shell uname -m)
URL:=https://github.com/helix-editor/helix/releases/download/25.01.1/helix_25.1.1-1_amd64.deb

.PHONY: build
build:
	@rm -rf build
	@mkdir build
ifeq ($(ARCH),x86_64)
	@echo "Building Docker image for architecture $(ARCH) with URL=$(URL)"
	@docker build --build-arg URL=${URL} -t helix:latest .
	docker cp $$(docker create --rm helix:latest):/helix/ "./build"

else
	@echo "Skipping build: Unsupported architecture $(ARCH)"
endif

.PHONY: clean
clean:
	@rm -rf build
	@docker rmi -f helix:latest || true
