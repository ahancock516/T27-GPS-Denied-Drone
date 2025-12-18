# .PHONY tells Make these are commands, not files
.PHONY: all clean build watch

# Default: Start everything (Normal startup)
all:
	@chmod +x run.sh
	@sudo ./run.sh

# Start everything + Force Rebuild (docker compose up --build)
build:
	@chmod +x run.sh
	@sudo ./run.sh --build

# Start in Development/Watch Mode (docker compose up --watch)
watch:
	@chmod +x run.sh
	@sudo ./run.sh --watch --build

# Stop and remove containers (docker compose down)
clean:
	@sudo docker compose down

# Pass-through: Allows 'make camera', 'make mavros', etc.
%:
	@chmod +x run.sh
	@sudo ./run.sh $@