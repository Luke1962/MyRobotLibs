// ///////////////////////////////////////////////////////////////////////////////
//  CHARARRAY STRING LIB					     ///////////////////////////////// 
// ///////////////////////////////////////////////////////////////////////////////
#pragma region Funzioni di manipolazione stringhe di tipo CHARARRAY
void strtrim(char* str) {
	int start = 0; // number of leading spaces
	char* buffer = str;
	while (*str && *str++ == ' ') ++start;
	while (*str++); // move to end of string
	int end = str - buffer - 1;
	while (end > 0 && buffer[end - 1] == ' ') --end; // backup over trailing spaces
	buffer[end] = 0; // remove trailing spaces
	if (end <= start || start == 0) return; // exit if no leading spaces or string is now empty
	str = buffer + start;
	while ((*buffer++ = *str++));  // remove leading spaces: K&R
}

ptrdiff_t index_of(const char *string, char search) {
	const char *moved_string = strchr(string, search);
	/* If not null, return the difference. */
	if (moved_string) {
		return moved_string - string;
	}
	/* Character not found. */
	return -1;
}

void substring(char *buff, byte pos, byte len) {
	char subbuff[50];
	memcpy(subbuff, &buff[pos], len);
	subbuff[len] = '\0';
}

template<size_t charCount>
void strtrim_safe(char(&output)[charCount]) {
	char *ptr = output;
	size_t n = charCount;
	size_t start = 0;
	size_t end = 0;

	// Find the start and end position of trimmed string
	while (n-- != 0 && *ptr != 0) {
		if (*ptr == 32) {
			if (end == 0) {
				start++;
			}
			else {
				break;
			}
		}
		else {
			end++;
		}

		ptr++;
	}

	// Shift the char array 
	for (int i = start, j = 0; i < end + start && j < charCount; i++, j++) {
		output[j] = output[i];
	}
	output[end] = 0;
}




#pragma endregion
#pragma endregion