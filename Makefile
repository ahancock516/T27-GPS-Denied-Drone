all: run

run:
	@chmod +x run.sh
	@sudo ./run.sh

clean:
	@sudo docker-compose down