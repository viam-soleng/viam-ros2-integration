module.tar.gz:
	tar czf $@ run.sh .env main.py components proto sample_configs services test_utils utils requirements.txt
