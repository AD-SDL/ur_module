
checks: # Runs all the pre-commit checks
	@pre-commit install
	@pre-commit run --all-files || { echo "Checking fixes\n" ; pre-commit run --all-files; }

test: init .env paths # Runs all the tests
	@docker compose -f wei.compose.yaml --env-file .env up --build -d
	@docker compose -f wei.compose.yaml --env-file .env exec ur_module pytest -p no:cacheprovider ur_module
	@docker compose -f wei.compose.yaml --env-file .env down
