# ai-world-constitution-docs/static/projects/bias_detector/bias_detector.py

def detect_gender_bias(sentence):
    # Simple list of stereotypically gendered words in professional contexts
    male_biased_words = {'he', 'him', 'his', 'man', 'men', 'chairman', 'businessman', 'spokesman'}
    female_biased_words = {'she', 'her', 'hers', 'woman', 'women', 'chairwoman', 'businesswoman', 'spokeswoman'}

    # More neutral alternatives or general biased terms
    general_biased_words = {'bossy', 'emotional', 'aggressive', 'rational'} # Context-dependent

    words = sentence.lower().split()
    found_biases = []

    for word in words:
        if word in male_biased_words:
            found_biases.append(f"Potential male bias: '{word}'")
        elif word in female_biased_words:
            found_biases.append(f"Potential female bias: '{word}'")
        elif word in general_biased_words:
            found_biases.append(f"Potential general bias (context-dependent): '{word}'")

    if not found_biases:
        return "No obvious gender-biased words detected in this sentence."
    else:
        return "\n".join(found_biases)


if __name__ == "__main__":
    print("--- Simple Gender Bias Detector ---")

    test_sentences = [
        "The chairman led the meeting effectively.",
        "The businesswoman managed her team with precision.",
        "Every engineer should check his code.",
        "A good manager is rational and objective.",
        "She was too emotional to make a clear decision."
    ]

    for i, sentence in enumerate(test_sentences):
        print(f"\nSentence {i+1}: '{sentence}'")
        result = detect_gender_bias(sentence)
        print(f"Analysis: {result}")

    print("\n--- Try your own sentences! ---")
    while True:
        user_input = input("Enter a sentence (or 'quit' to exit): ")
        if user_input.lower() == 'quit':
            break
        result = detect_gender_bias(user_input)
        print(f"Analysis: {result}")

