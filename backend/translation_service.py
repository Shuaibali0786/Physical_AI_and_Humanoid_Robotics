"""
Translation Service for English to Roman Urdu
Provides translation functionality that can be used by the RAG system
"""
import logging
import re
from typing import Dict, List, Optional
from dataclasses import dataclass

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class TranslationResult:
    original_text: str
    translated_text: str
    success: bool
    error: Optional[str] = None

class TranslationService:
    def __init__(self):
        # Basic English to Roman Urdu transliteration mappings
        # This is a simplified mapping - in a real implementation, you'd want a more comprehensive dictionary
        self.transliteration_map = {
            # Vowels
            'a': 'a', 'e': 'e', 'i': 'i', 'o': 'o', 'u': 'u',
            # Consonants
            'b': 'b', 'c': 'c', 'd': 'd', 'f': 'f', 'g': 'g',
            'h': 'h', 'j': 'j', 'k': 'k', 'l': 'l', 'm': 'm',
            'n': 'n', 'p': 'p', 'q': 'q', 'r': 'r', 's': 's',
            't': 't', 'v': 'v', 'w': 'w', 'x': 'x', 'y': 'y', 'z': 'z',
            # Common English words to Roman Urdu equivalents
            'the': 'de', 'and': 'aur', 'or': 'ya', 'is': 'hai', 'are': 'hain',
            'was': 'tha', 'were': 'the', 'be': 'ho', 'been': 'hua', 'have': 'hai',
            'has': 'hai', 'had': 'tha', 'do': 'karna', 'does': 'karta', 'did': 'kiya',
            'will': 'hoga', 'would': 'karta tha', 'should': 'chahiye', 'can': 'sakta',
            'could': 'sakta tha', 'may': 'ho sakta', 'might': 'ho sakta tha',
            'must': 'chahiye', 'shall': 'hoga',
            # Additional common words
            'in': 'mein', 'on': 'par', 'at': 'par', 'to': 'ko', 'for': 'ke liye',
            'of': 'ka', 'with': 'ke saath', 'by': 'ke dwara', 'from': 'se',
            'up': 'upar', 'down': 'neeche', 'out': 'bahar', 'off': 'band',
            'over': 'upar', 'under': 'neeche', 'again': 'phir', 'further': 'aage',
            'then': 'phir', 'once': 'ek baar', 'here': 'yahan', 'there': 'wahan',
            'when': 'jab', 'where': 'kahan', 'why': 'kyun', 'how': 'kaise',
            'all': 'sab', 'any': 'koi', 'both': 'donon', 'each': 'pratyek',
            'few': 'kam', 'more': 'zada', 'most': 'adhik', 'other': 'doosra',
            'some': 'kuch', 'such': 'aisa', 'no': 'nahi', 'nor': 'na',
            'not': 'nahi', 'only': 'sirf', 'own': 'apna', 'same': 'same',
            'so': 'isliye', 'than': 'se', 'too': 'bhi', 'very': 'bahut',
            'just': 'sirf', 'now': 'ab', 'today': 'aaj', 'yesterday': 'kal',
            'tomorrow': 'kal', 'week': 'hafta', 'month': 'mahina', 'year': 'saal',
            'day': 'din', 'time': 'samay', 'man': 'aadmi', 'woman': 'aurat',
            'child': 'bachcha', 'people': 'log', 'way': 'tareeka', 'work': 'kaam',
            'life': 'jeevan', 'hand': 'haath', 'eye': 'aankh', 'head': 'sir',
            'house': 'ghar', 'water': 'paani', 'food': 'khaana', 'book': 'kitab',
            'school': 'sakool', 'student': 'chela', 'teacher': 'aadhyaapak',
            'computer': 'kampyootar', 'phone': 'phone', 'car': 'gaadi',
            'road': 'sarak', 'city': 'sheher', 'country': 'desh', 'world': 'duniya',
            'good': 'achha', 'bad': 'bura', 'big': 'bara', 'small': 'chota',
            'new': 'naya', 'old': 'purana', 'right': 'sahi', 'wrong': 'galat',
            'yes': 'haan', 'no': 'nahi', 'hello': 'namaste', 'hi': 'haanji',
            'bye': 'alvida', 'thank': 'shukriya', 'please': 'krpaye',
            'sorry': 'maafi', 'love': 'pyaar', 'hate': 'nafrat', 'like': 'pasand',
            'want': 'chaahata', 'need': 'jaroorat', 'help': 'madad', 'know': 'jaanta',
            'think': 'samajhta', 'see': 'dekhna', 'hear': 'sunna', 'speak': 'baat',
            'read': 'padhna', 'write': 'likhna', 'go': 'jaana', 'come': 'ana',
            'give': 'dena', 'take': 'lena', 'make': 'banana', 'get': 'paana',
            'use': 'upayog', 'find': 'dhundhna', 'tell': 'bataana', 'ask': 'poochhna',
            'try': 'koshish', 'call': 'pukaarna', 'feel': 'mehsus', 'become': 'banana',
            'leave': 'chhodna', 'put': 'rakhna', 'mean': 'matlab', 'keep': 'rakhna',
            'let': 'dena', 'begin': 'shuru', 'seem': 'lagta', 'help': 'madad',
            'show': 'dikhana', 'hear': 'sunna', 'play': 'khelna', 'run': 'daudna',
            'walk': 'chalna', 'stand': 'khada', 'sit': 'baithna', 'sleep': 'sona',
            'eat': 'khana', 'drink': 'peena', 'buy': 'khareedna', 'sell': 'bechna',
            'open': 'khulna', 'close': 'bandhna', 'turn': 'mudna', 'start': 'shuru',
            'stop': 'rukna', 'turn': 'mudna', 'move': 'hilkna', 'live': 'raihna',
            'die': 'marjana', 'kill': 'maarna', 'hurt': 'chot', 'break': 'tutna',
            'cut': 'kaatna', 'fall': 'girna', 'rise': 'uthna', 'grow': 'badhna',
            'fall': 'girna', 'sit': 'baithna', 'rise': 'uthna', 'wash': 'dhona',
            'clean': 'saaf', 'dirty': 'ganda', 'hot': 'garam', 'cold': 'thanda',
            'warm': 'garam', 'cool': 'thanda', 'fast': 'tez', 'slow': 'dhire',
            'early': 'jaldi', 'late': 'der', 'long': 'lama', 'short': 'chota',
            'high': 'uncha', 'low': 'neeche', 'deep': 'gehra', 'shallow': 'uthala',
            'wide': 'chaudha', 'narrow': 'tankha', 'thick': 'moti', 'thin': 'patli',
            'heavy': 'bhari', 'light': 'halka', 'strong': 'mazboot', 'weak': 'kamaz',
            'young': 'jawaan', 'old': 'budha', 'fresh': 'taza', 'stale': 'badboodar',
            'rough': 'khara', 'smooth': 'chikna', 'sharp': 'tez', 'dull': 'kanak',
            'loose': 'ढila', 'tight': 'kasakar', 'dry': 'sookha', 'wet': 'gila',
            'hard': 'kathor', 'soft': 'naram', 'lovely': 'pyaara', 'ugly': 'kharab',
            'clever': 'chatur', 'foolish': 'bevkoof', 'brave': 'bheekh', 'coward': 'kayar',
            'honest': 'sachha', 'dishonest': 'jhootha', 'polite': 'sharif', 'rude': 'bada',
            'true': 'sach', 'false': 'jhooth', 'real': 'asli', 'fake': 'nakli',
            'possible': 'sambhav', 'impossible': 'asambhav', 'easy': 'aasaan', 'difficult': 'kathin',
            'safe': 'surakshit', 'dangerous': 'khatarnaak', 'useful': 'upayogi', 'useless': 'bekaar',
            'important': 'mahatvapoorn', 'unimportant': 'anupayogi', 'interesting': 'dilchasp', 'boring': 'ukhada',
            'funny': 'mazedaar', 'serious': 'gambheer', 'happy': 'khush', 'sad': 'udaas',
            'angry': 'gussa', 'calm': 'shaant', 'excited': 'uttejita', 'bored': 'ubha',
            'tired': 'thakka', 'energetic': 'shaktishali', 'healthy': 'swasth', 'sick': 'beemar',
            'rich': 'aamir', 'poor': 'garib', 'beautiful': 'sundar', 'handsome': 'husn',
            'pretty': 'sundar', 'clean': 'saaf', 'dirty': 'ganda', 'holy': 'paavak',
            'evil': 'bhoot', 'good': 'achha', 'bad': 'bura', 'right': 'sahi', 'wrong': 'galat',
            'correct': 'sahee', 'incorrect': 'galat', 'accurate': 'sahee', 'inaccurate': 'galat',
            'complete': 'poora', 'incomplete': 'aadhura', 'perfect': 'kamal', 'imperfect': 'nakis',
            'sufficient': 'paryaapt', 'insufficient': 'aparyaapt', 'adequate': 'paryaapt', 'inadequate': 'aparyaapt',
            'satisfactory': 'santoshajak', 'unsatisfactory': 'asantoshajak', 'acceptable': 'swikarya', 'unacceptable': 'aswikarya',
            'available': 'upalabdh', 'unavailable': 'anupalabdh', 'present': 'upasthit', 'absent': 'anupasthit',
            'included': 'shamil', 'excluded': 'hata diya', 'contained': 'samanvit', 'not contained': 'na samanvit',
            'same': 'same', 'different': 'alag', 'similar': 'jaisa', 'unlike': 'alag',
            'like': 'jaisa', 'as': 'jaise', 'than': 'se', 'from': 'se', 'to': 'ko',
            'into': 'mein', 'onto': 'par', 'upon': 'par', 'about': 'ke baare mein',
            'above': 'upar', 'below': 'neeche', 'across': 'par', 'through': 'ke dvaara',
            'around': 'charound', 'before': 'pehle', 'after': 'baad', 'behind': 'peeche',
            'between': 'beech mein', 'among': 'beech mein', 'beneath': 'neeche', 'underneath': 'neeche',
            'near': 'paas', 'next': 'aage', 'beside': 'ke baajoo', 'opposite': 'vilom',
            'outside': 'bahar', 'inside': 'andhar', 'within': 'andhar', 'throughout': 'poore samay',
            'towards': 'ki aur', 'away': 'door', 'off': 'band', 'on': 'chalu',
            'over': 'upar', 'past': 'guzar', 'since': 'se', 'until': 'tak',
            'by': 'ke dwara', 'via': 'ke dwara', 'per': 'prati', 'without': 'bina',
            'with': 'ke saath', 'through': 'ke dvaara', 'during': 'ke dauraan', 'for': 'ke liye',
            'about': 'ke baare mein', 'of': 'ka', 'at': 'par', 'in': 'mein',
            'on': 'par', 'to': 'ko', 'from': 'se', 'by': 'ke dwara',
            'I': 'main', 'you': 'aap', 'he': 'vo', 'she': 'vo', 'it': 'ye',
            'we': 'ham', 'they': 'vo', 'me': 'mujhe', 'him': 'usko', 'her': 'usko',
            'us': 'humko', 'them': 'unko', 'my': 'mera', 'your': 'aapka', 'his': 'uska',
            'her': 'uski', 'its': 'iska', 'our': 'hamara', 'their': 'unka', 'mine': 'mera',
            'yours': 'aapka', 'his': 'uska', 'hers': 'uski', 'ours': 'hamara', 'theirs': 'unka',
            'this': 'yah', 'that': 'vah', 'these': 'ye', 'those': 'vo', 'am': 'hun',
            'is': 'hai', 'are': 'hain', 'was': 'tha', 'were': 'the', 'be': 'ho',
            'been': 'hua', 'being': 'hota', 'have': 'hai', 'has': 'hai', 'had': 'tha',
            'do': 'karna', 'does': 'karta', 'did': 'kiya', 'will': 'hoga', 'would': 'karta tha',
            'should': 'chahiye', 'can': 'sakta', 'could': 'sakta tha', 'may': 'ho sakta',
            'might': 'ho sakta tha', 'must': 'chahiye', 'shall': 'hoga', 'shall': 'hoga'
        }

        # Create reverse mapping for some common words
        self.reverse_transliteration_map = {v: k for k, v in self.transliteration_map.items()}

    def transliterate_to_roman_urdu(self, text: str) -> str:
        """
        Convert English text to Roman Urdu using basic transliteration rules.

        Args:
            text: The English text to transliterate

        Returns:
            Transliterated text in Roman Urdu
        """
        if not text:
            return ""

        # Convert to lowercase for processing
        text = text.lower()

        # Split text into words and process each word
        words = text.split()
        transliterated_words = []

        for word in words:
            # Remove punctuation for processing but preserve it
            clean_word = re.sub(r'[^\w\s]', '', word)
            punctuation = re.sub(r'[\w\s]', '', word)

            if clean_word in self.transliteration_map:
                transliterated_word = self.transliteration_map[clean_word]
            else:
                # Apply basic phonetic transliteration for unknown words
                transliterated_word = self._basic_phonetic_transliteration(clean_word)

            # Add back punctuation
            transliterated_word += punctuation
            transliterated_words.append(transliterated_word)

        return ' '.join(transliterated_words)

    def _basic_phonetic_transliteration(self, word: str) -> str:
        """
        Apply basic phonetic transliteration rules for unknown words.

        Args:
            word: The word to transliterate

        Returns:
            Transliterated word
        """
        # For this implementation, we'll use a simple approach
        # In a real implementation, you'd want more sophisticated phonetic rules
        if not word:
            return word

        # Basic vowel handling
        word = re.sub(r'ea', 'i', word)
        word = re.sub(r'ee', 'i', word)
        word = re.sub(r'oo', 'u', word)
        word = re.sub(r'ou', 'aw', word)
        word = re.sub(r'ow', 'au', word)
        word = re.sub(r'ai', 'ae', word)
        word = re.sub(r'ay', 'e', word)
        word = re.sub(r'oy', 'oi', word)
        word = re.sub(r'ie', 'i', word)
        word = re.sub(r'ch', 'ch', word)
        word = re.sub(r'th', 'th', word)
        word = re.sub(r'sh', 'sh', word)
        word = re.sub(r'ph', 'f', word)
        word = re.sub(r'qu', 'kw', word)
        word = re.sub(r'x', 'ks', word)
        word = re.sub(r'z', 'z', word)

        # Convert remaining letters as-is
        result = ""
        for char in word:
            if char.isalpha():
                result += char
            else:
                result += char

        return result

    def transliterate_from_roman_urdu(self, text: str) -> str:
        """
        Convert Roman Urdu back to English (reverse transliteration).

        Args:
            text: The Roman Urdu text to convert back to English

        Returns:
            English text
        """
        if not text:
            return ""

        # Split text into words and process each word
        words = text.split()
        english_words = []

        for word in words:
            # Remove punctuation for processing but preserve it
            clean_word = re.sub(r'[^\w\s]', '', word)
            punctuation = re.sub(r'[\w\s]', '', word)

            if clean_word in self.reverse_transliteration_map:
                english_word = self.reverse_transliteration_map[clean_word]
            else:
                # For unknown words, keep as is
                english_word = clean_word

            # Add back punctuation
            english_word += punctuation
            english_words.append(english_word)

        return ' '.join(english_words)

    def translate_text(self, text: str, target_language: str = "roman_urdu") -> TranslationResult:
        """
        Translate text to the target language.

        Args:
            text: The text to translate
            target_language: The target language ("roman_urdu" or "english")

        Returns:
            TranslationResult containing the translated text
        """
        try:
            if target_language.lower() in ["roman_urdu", "romanurdu", "urdu"]:
                translated_text = self.transliterate_to_roman_urdu(text)
            elif target_language.lower() == "english":
                translated_text = self.transliterate_from_roman_urdu(text)
            else:
                return TranslationResult(
                    original_text=text,
                    translated_text="",
                    success=False,
                    error=f"Unsupported target language: {target_language}"
                )

            return TranslationResult(
                original_text=text,
                translated_text=translated_text,
                success=True
            )
        except Exception as e:
            logger.error(f"Error translating text: {e}")
            return TranslationResult(
                original_text=text,
                translated_text="",
                success=False,
                error=str(e)
            )

    def translate_content_batch(self, texts: List[str], target_language: str = "roman_urdu") -> List[TranslationResult]:
        """
        Translate multiple texts in batch.

        Args:
            texts: List of texts to translate
            target_language: The target language ("roman_urdu" or "english")

        Returns:
            List of TranslationResult objects
        """
        results = []
        for text in texts:
            result = self.translate_text(text, target_language)
            results.append(result)
        return results


# Singleton instance
translation_service = TranslationService()


def translate_text(text: str, target_language: str = "roman_urdu") -> TranslationResult:
    """
    Public function to translate text.

    Args:
        text: The text to translate
        target_language: The target language ("roman_urdu" or "english")

    Returns:
        TranslationResult containing the translated text
    """
    return translation_service.translate_text(text, target_language)