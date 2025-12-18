# Default: If you just type 'make', run everything
all:
	@chmod +x run.sh
	@sudo ./run.sh

# Stop everything
clean:
	@sudo docker compose down

# Catch-All: matches anything else you type (e.g., 'make camera')
%:
	@chmod +x run.sh
	@sudo ./run.sh $@