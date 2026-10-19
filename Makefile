PYTHON ?= python3

.PHONY: i18n-extract i18n-apply i18n-verify i18n-test i18n-sync

i18n-extract:
	$(PYTHON) tools/i18n/extract_tm.py --base main --localized zh --out i18n/tm

i18n-apply:
	$(PYTHON) tools/i18n/apply_tm.py --source-dir . --tm i18n/tm

i18n-verify:
	$(PYTHON) tools/i18n/verify.py --base-ref main --localized-ref zh

i18n-test:
	$(PYTHON) -m unittest discover -s tools/i18n/tests -v

i18n-sync:
	./tools/i18n/sync.sh
