"""
Test the RAG agent directly to see what's happening.
"""

import os
from dotenv import load_dotenv
from rag_agent import agent
from agents import Runner

load_dotenv()

print("=" * 60)
print("RAG Agent Test")
print("=" * 60)

test_queries = [
    "How do I install ROS 2?",
    "What is Physical AI?",
    "Tell me about Gazebo"
]

for query in test_queries:
    print(f"\nQuery: '{query}'")
    print("-" * 60)

    try:
        # Run the agent
        result = Runner.run_sync(agent, input=query)

        print(f"Response type: {type(result)}")
        print(f"Response: {result}")

        # Try to extract final output
        if hasattr(result, 'final_output'):
            print(f"Final Output: {result.final_output}")

        # Try to see messages/steps
        if hasattr(result, 'messages'):
            print(f"Messages: {result.messages}")

        # Print all attributes
        print(f"Attributes: {dir(result)}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

print("\n" + "=" * 60)
