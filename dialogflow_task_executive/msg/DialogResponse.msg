Header header

# query: The text send by user
string query            # The query created by user

# action, response, parameters: The response from Dialogflow
string action           # Inferred action name, which is registered at Dialogflow config
                        # Dialogflow has Default Fallback Intent by default, and it returns input.unknown as action name, when it gets unknown queries
string response         # The response from Dialogflow based on action name
string parameters       # The action's parameters

# fulfilled, speech_score, intent_score: Parameters related to results
bool fulfilled          # Whether all required parameters are present
float32 speech_score    # Speech recognition confidence
float32 intent_score    # Intent detection confidence
