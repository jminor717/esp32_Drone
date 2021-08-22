
/**
 * @brief calculate the PJW hash for a given char array
 *
 * @note adapted from https://en.wikipedia.org/wiki/PJW_hash_function
 *
 * @return
 *      - 32 bit hash for the given input data
 */
static unsigned int ElfHash(const unsigned char *s, unsigned int len);