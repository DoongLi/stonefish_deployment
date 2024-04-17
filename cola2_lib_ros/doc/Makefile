.DEFAULT_GOAL := default

clean:
	rm -rf _build html/cpp html/python python

default:
	# clean
	rm -rf _build html/cpp html/python python
	# cpp
	mkdir -p html/cpp
	doxygen Doxyfile
	# python
	sphinx-apidoc -f -o python ../python/cola2_ros
	sphinx-build -M html . _build
	mkdir -p html/python
	cp -r _build/html/* html/python/
	# @$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
