import json
import jsonschema
from jsonschema import validate

# File paths
schema_file_path = "/Users/mwweiss/blossom-exp/blossom-sdk-main/Sequences/sequence_schema.json"  # Path to JSON Schema file
json_file_path = "/Users/mwweiss/blossom-exp/blossom-sdk-main/Sequences/tiny_test.json"  # Path to JSON file

try:
    # Load JSON Schema from file
    with open(schema_file_path, "r") as schema_file:
        schema = json.load(schema_file)

    # Load JSON data from file
    with open(json_file_path, "r") as json_file:
        json_data = json.load(json_file)

    # Validate JSON against the schema
    validate(instance=json_data, schema=schema)

    print("JSON is valid against the schema!")

except jsonschema.exceptions.ValidationError as e:
    print("JSON validation error:", e.message)

except jsonschema.exceptions.SchemaError as e:
    print("Schema error:", e.message)

except FileNotFoundError as e:
    print("File not found:", e)

except json.JSONDecodeError as e:
    print("Invalid JSON format:", e)

except Exception as e:
    print("Unexpected error:", e)