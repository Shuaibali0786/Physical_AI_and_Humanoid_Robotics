/**
 * English to Roman Urdu Transliteration Utility
 *
 * This utility converts English text to Roman Urdu script using a mapping approach.
 * Roman Urdu is the representation of Urdu words using Latin script with additional
 * characters to represent sounds unique to Urdu.
 */

// English to Roman Urdu character mapping
// Roman Urdu uses Latin alphabet with some modifications to represent Urdu sounds
const ENGLISH_TO_URDU_MAP: Record<string, string> = {
  // Vowels - Roman Urdu representation
  'a': 'a', 'e': 'e', 'i': 'i', 'o': 'o', 'u': 'u',
  'A': 'A', 'E': 'E', 'I': 'I', 'O': 'O', 'U': 'U',

  // Consonants - Roman Urdu representation
  'b': 'b', 'c': 'c', 'd': 'd', 'f': 'f', 'g': 'g',
  'h': 'h', 'j': 'j', 'k': 'k', 'l': 'l', 'm': 'm',
  'n': 'n', 'p': 'p', 'q': 'q', 'r': 'r', 's': 's',
  't': 't', 'v': 'v', 'w': 'w', 'x': 'x', 'y': 'y', 'z': 'z',
  'B': 'B', 'C': 'C', 'D': 'D', 'F': 'F', 'G': 'G',
  'H': 'H', 'J': 'J', 'K': 'K', 'L': 'L', 'M': 'M',
  'N': 'N', 'P': 'P', 'Q': 'Q', 'R': 'R', 'S': 'S',
  'T': 'T', 'V': 'V', 'W': 'W', 'X': 'X', 'Y': 'Y', 'Z': 'Z',

  // Common digraphs and trigraphs that represent specific Urdu sounds
  'th': 'th',  // For aspirated sounds
  'Th': 'Th',
  'TH': 'TH',
  'dh': 'dh',
  'Dh': 'Dh',
  'DH': 'DH',
  'ph': 'ph',
  'Ph': 'Ph',
  'PH': 'PH',
  'kh': 'kh',
  'Kh': 'Kh',
  'KH': 'KH',
  'sh': 'sh',
  'Sh': 'Sh',
  'SH': 'SH',
  'ch': 'ch',
  'Ch': 'Ch',
  'CH': 'CH',
  'jh': 'jh',
  'Jh': 'Jh',
  'JH': 'JH',
  'gh': 'gh',
  'Gh': 'Gh',
  'GH': 'GH',
  'ng': 'ng',
  'Ng': 'Ng',
  'NG': 'NG',
  'bh': 'bh',
  'Bh': 'Bh',
  'BH': 'BH',
};

// Common English words to Roman Urdu mapping for better accuracy
const COMMON_WORDS_MAP: Record<string, string> = {
  'the': 'the',
  'and': 'and',
  'or': 'ya',
  'but': 'lekin',
  'for': 'ke liye',
  'to': 'ko',
  'in': 'mein',
  'on': 'par',
  'at': 'par',
  'is': 'hai',
  'are': 'hain',
  'was': 'tha',
  'were': 'the',
  'be': 'ho',
  'been': 'hua',
  'have': 'hai',
  'has': 'hai',
  'had': 'tha',
  'do': 'karna',
  'does': 'karta',
  'did': 'kiya',
  'will': 'hoga',
  'would': 'hota',
  'could': 'sakta',
  'should': 'chahiye',
  'can': 'sakta',
  'may': 'ho sakta',
  'might': 'ho sakta',
  'hello': 'assalam o alaikum',
  'hi': 'ao',
  'goodbye': 'khuda hafiz',
  'yes': 'haan',
  'no': 'nhi',
  'please': 'meherbani',
  'thank': 'shukriya',
  'thanks': 'shukriya',
  'sorry': 'maafi',
  'love': 'mohabbat',
  'time': 'waqt',
  'day': 'din',
  'night': 'raat',
  'water': 'paani',
  'food': 'khaana',
  'house': 'ghar',
  'work': 'kaam',
  'home': 'ghar',
  'school': 'maktab',
  'book': 'kitab',
  'friend': 'dost',
  'family': 'khandaan',
  'mother': 'maa',
  'father': 'baap',
  'brother': 'bhai',
  'sister': 'bhen',
  'person': 'shakhs',
  'man': 'aadam',
  'woman': 'aurat',
  'child': 'bacha',
  'children': 'bachay',
  'name': 'naam',
  'country': 'mulk',
  'city': 'shehar',
  'world': 'duniya',
  'life': 'zindagi',
  'god': 'khuda',
  'religion': 'mazhab',
  'faith': 'imaan',
  'truth': 'haqeeqat',
  'peace': 'aaman',
  'war': 'jang',
  'education': 'taleem',
  'knowledge': 'ilm',
  'science': 'saiyansi',
  'technology': 'taknology',
  'computer': 'kamputer',
  'internet': 'intarnet',
  'mobile': 'mobil',
  'phone': 'phone',
  'car': 'kar',
  'bus': 'bas',
  'train': 'train',
  'plane': 'plain',
  'money': 'paisa',
  'work': 'kaam',
  'job': 'naukri',
  'money': 'paisa',
  'price': 'daam',
  'market': 'baazar',
  'shop': 'dukaan',
  'buy': 'khareedna',
  'sell': 'bechna',
  'eat': 'khana',
  'drink': 'peena',
  'walk': 'chalna',
  'run': 'daudna',
  'sleep': 'sona',
  'wake': 'jagna',
  'see': 'dekhna',
  'hear': 'sunna',
  'speak': 'bolna',
  'think': 'sochna',
  'feel': 'mehsus',
  'know': 'jaanna',
  'learn': 'seekhna',
  'teach': 'pudhna',
  'read': 'padhna',
  'write': 'likhna',
  'draw': 'khechna',
  'sing': 'gaana',
  'dance': 'naachna',
  'play': 'khelna',
  'work': 'kaam karna',
  'help': 'madad',
  'give': 'dena',
  'take': 'lena',
  'bring': 'lana',
  'go': 'jana',
  'come': 'ana',
  'stay': 'rahna',
  'leave': 'jana',
  'return': 'wapis',
  'enter': 'ander jana',
  'exit': 'baahir jana',
  'up': 'upar',
  'down': 'neechay',
  'left': 'baein',
  'right': 'daein',
  'front': 'aagey',
  'back': 'peechay',
  'near': 'paas',
  'far': 'door',
  'here': 'yahan',
  'there': 'wahan',
  'this': 'yeh',
  'that': 'woh',
  'these': 'ye',
  'those': 'wo',
  'who': 'kaun',
  'what': 'kya',
  'where': 'kahan',
  'when': 'kab',
  'why': 'kyun',
  'how': 'kaisy',
  'which': 'kaunsa',
  'whose': 'kiska',
  'my': 'mera',
  'your': 'aapka',
  'his': 'uska',
  'her': 'uski',
  'its': 'uska',
  'our': 'hamara',
  'their': 'unka',
  'one': 'ek',
  'two': 'do',
  'three': 'teeen',
  'four': 'chaar',
  'five': 'paanch',
  'six': 'cheh',
  'seven': 'saat',
  'eight': 'aath',
  'nine': 'nau',
  'ten': 'das',
  'first': 'pehla',
  'last': 'aakhri',
  'big': 'bara',
  'small': 'chota',
  'long': 'lama',
  'short': 'chota',
  'tall': 'bulanda',
  'high': 'bulanda',
  'low': 'kam',
  'deep': 'gehra',
  'shallow': 'thoka',
  'fast': 'tez',
  'slow': 'dhire',
  'hot': 'garam',
  'cold': 'thanda',
  'warm': 'garam',
  'cool': 'thanda',
  'old': 'purana',
  'new': 'naya',
  'young': 'jawaan',
  'good': 'acha',
  'bad': 'beha',
  'right': 'sahi',
  'wrong': 'galat',
  'true': 'sahi',
  'false': 'jhooth',
  'real': 'asli',
  'fake': 'nakli',
  'open': 'khulna',
  'close': 'band karna',
  'light': 'roshni',
  'dark': 'andhera',
  'loud': 'tez',
  'quiet': 'chup',
  'hard': 'sakhta',
  'soft': 'naram',
  'rough': 'kharra',
  'smooth': 'chikna',
  'full': 'bhara',
  'empty': 'khali',
  'heavy': 'bhara',
  'light': 'halka',
  'strong': 'mazboot',
  'weak': 'kamzor',
  'big': 'bara',
  'little': 'thoka',
  'large': 'bara',
  'great': 'bara',
  'small': 'chota',
  'tiny': 'nano',
  'clean': 'saaf',
  'dirty': 'ganda',
  'wet': 'gila',
  'dry': 'khushk',
  'clean': 'saaf',
  'beautiful': 'khubsurat',
  'ugly': 'qabih',
  'pretty': 'sundar',
  'handsome': 'husn',
  'happy': 'khush',
  'sad': 'gham',
  'angry': 'gusse',
  'calm': 'aaram',
  'nervous': 'chid',
  'brave': 'jasoor',
  'afraid': 'dar',
  'proud': 'fakhar',
  'shy': 'sharam',
  'polite': 'tameez',
  'rude': 'beadbi',
  'honest': 'sidha',
  'cruel': 'zulm',
  'kind': 'mehar',
  'mean': 'bukhar',
  'generous': 'kheech',
  'selfish': 'khudgarz',
  'patient': 'sabur',
  'impatient': 'bechain',
  'careful': 'hoshyaar',
  'careless': 'laaparwah',
  'hardworking': 'mehnat',
  'lazy': 'aali',
  'humble': 'fakiri',
  'arrogant': 'takabur',
  'modest': 'sharam',
  'bold': 'jasoor',
  'timid': 'kharab',
  'wise': 'hakeem',
  'foolish': 'bewqoof',
  'intelligent': 'aqal',
  'stupid': 'bewqoof',
  'clever': 'chalaak',
  'dull': 'kanjar',
  'interesting': 'daulat',
  'boring': 'ub',
  'exciting': 'khushi',
  'amazing': 'ajeeb',
  'wonderful': 'ajeeb',
  'fantastic': 'ajeeb',
  'terrible': 'bura',
  'awful': 'kharab',
  'horrible': 'darr',
  'frightening': 'dar',
  'scary': 'dar',
  'frightened': 'dar',
  'scared': 'dar',
  'fearful': 'dar',
  'nervous': 'chid',
  'anxious': 'chid',
  'worried': 'fikar',
  'concerned': 'fikar',
  'interested': 'dilchaspi',
  'curious': 'jugnoo',
  'surprised': 'hairan',
  'amazed': 'ajeeb',
  'astonished': 'hairan',
  'shocked': 'hairan',
  'surprising': 'hairan',
  'amazing': 'ajeeb',
  'incredible': 'ajeeb',
  'unbelievable': 'yaqeen',
  'impossible': 'namumkin',
  'possible': 'mumkin',
  'probable': 'imkan',
  'likely': 'imkan',
  'unlikely': 'naqil',
  'certain': 'yaqeen',
  'uncertain': 'naqil',
  'sure': 'yaqeen',
  'unsure': 'naqil',
  'clear': 'saf',
  'unclear': 'ghalat',
  'obvious': 'saf',
  'evident': 'saf',
  'apparent': 'saf',
  'visible': 'nazar',
  'invisible': 'nazar',
  'present': 'haazir',
  'absent': 'ghair',
  'available': 'mil',
  'unavailable': 'na mil',
  'ready': 'taiyar',
  'unready': 'taiyar',
  'prepared': 'taiyar',
  'unprepared': 'taiyar',
  'willing': 'taiyar',
  'unwilling': 'taiyar',
  'able': 'sakht',
  'unable': 'na sakt',
  'capable': 'sakht',
  'incapable': 'na sakt',
  'competent': 'sakht',
  'incompetent': 'na sakt',
  'qualified': 'sakht',
  'unqualified': 'na sakt',
  'experienced': 'sakht',
  'inexperienced': 'na sakt',
  'trained': 'sakht',
  'untrained': 'na sakt',
  'skilled': 'sakht',
  'unskilled': 'na sakt',
  'talented': 'sakht',
  'untalented': 'na sakt',
  'gifted': 'sakht',
  'ungifted': 'na sakt',
  'natural': 'fitri',
  'artificial': 'sakht',
  'real': 'asli',
  'fake': 'nakli',
  'genuine': 'asli',
  'false': 'jhooth',
  'true': 'sahi',
  'right': 'sahi',
  'wrong': 'galat',
  'correct': 'sahi',
  'incorrect': 'galat',
  'accurate': 'sahi',
  'inaccurate': 'galat',
  'exact': 'sahi',
  'inexact': 'galat',
  'precise': 'sahi',
  'imprecise': 'galat',
  'proper': 'sahi',
  'improper': 'galat',
  'appropriate': 'sahi',
  'inappropriate': 'galat',
  'suitable': 'sahi',
  'unsuitable': 'galat',
  'fit': 'sahi',
  'unfit': 'galat',
  'right': 'sahi',
  'wrong': 'galat',
  'correct': 'sahi',
  'incorrect': 'galat',
  'valid': 'sahi',
  'invalid': 'galat',
  'legal': 'jaiz',
  'illegal': 'najaiz',
  'lawful': 'jaiz',
  'unlawful': 'najaiz',
  'permitted': 'jaiz',
  'forbidden': 'haram',
  'allowed': 'jaiz',
  'disallowed': 'najaiz',
  'permissible': 'jaiz',
  'impermissible': 'najaiz',
  'acceptable': 'qabil',
  'unacceptable': 'ghair',
  'tolerable': 'qabil',
  'intolerable': 'ghair',
  'bearable': 'qabil',
  'unbearable': 'ghair',
  'endurable': 'qabil',
  'unendurable': 'ghair',
  'manageable': 'qabil',
  'unmanageable': 'ghair',
  'controllable': 'qabil',
  'uncontrollable': 'ghair',
  'possible': 'mumkin',
  'impossible': 'namumkin',
  'feasible': 'mumkin',
  'infeasible': 'namumkin',
  'practicable': 'mumkin',
  'impracticable': 'namumkin',
  'workable': 'mumkin',
  'unworkable': 'namumkin',
  'viable': 'mumkin',
  'nonviable': 'namumkin',
  'practical': 'amali',
  'impractical': 'ghair',
  'useful': 'faida',
  'useless': 'beha',
  'helpful': 'madad',
  'unhelpful': 'beha',
  'beneficial': 'faida',
  'unbeneficial': 'beha',
  'advantageous': 'faida',
  'disadvantageous': 'beha',
  'profitable': 'faida',
  'unprofitable': 'beha',
  'worthwhile': 'faida',
  'worthless': 'beha',
  'valuable': 'qadar',
  'valueless': 'beha',
  'precious': 'qadar',
  'worthless': 'beha',
  'costly': 'mehar',
  'expensive': 'mehar',
  'cheap': 'sasta',
  'inexpensive': 'sasta',
  'dear': 'mehar',
  'dearest': 'mehar',
  'dearly': 'mehar',
  'cheaply': 'sasta',
  'expensively': 'mehar',
  'cost': 'mehar',
  'price': 'daam',
  'value': 'qadar',
  'worth': 'qadar',
  'expense': 'kharcha',
  'payment': 'ada',
  'money': 'paisa',
  'cash': 'naqd',
  'coin': 'sikka',
  'note': 'kaghaz',
  'bill': 'billa',
  'receipt': 'rasid',
  'account': 'hisab',
  'bank': 'bank',
  'loan': 'qarz',
  'credit': 'credit',
  'debit': 'debit',
  'interest': 'sood',
  'profit': 'faida',
  'loss': 'nuqsaan',
  'gain': 'faida',
  'earn': 'kamaana',
  'make': 'banana',
  'get': 'lena',
  'receive': 'milaana',
  'obtain': 'milaana',
  'acquire': 'hasil',
  'buy': 'khareedna',
  'purchase': 'khareedna',
  'sell': 'bechna',
  'offer': 'peshkash',
  'present': 'peshkash',
  'gift': 'peshkash',
  'donate': 'deena',
  'give': 'deena',
  'lend': 'deena',
  'borrow': 'lena',
  'owe': 'qarz',
  'owe': 'qarz',
  'owe': 'qarz',
  'owe': 'qarz',
};

/**
 * Transliterates English text to Roman Urdu
 * @param text The English text to transliterate
 * @returns The Roman Urdu transliteration
 */
export function transliterateToRomanUrdu(text: string): string {
  if (!text || typeof text !== 'string') {
    return '';
  }

  // First try to match common words
  const words = text.split(/\s+/);
  const result: string[] = [];

  for (const word of words) {
    // Check if the entire word exists in the common words map (case-insensitive)
    const lowerWord = word.toLowerCase();
    if (COMMON_WORDS_MAP.hasOwnProperty(lowerWord)) {
      result.push(COMMON_WORDS_MAP[lowerWord]);
    } else {
      // If not a common word, transliterate character by character
      let transliteratedWord = '';
      for (let i = 0; i < word.length; i++) {
        const char = word[i];
        transliteratedWord += ENGLISH_TO_URDU_MAP[char] || char;
      }
      result.push(transliteratedWord);
    }
  }

  return result.join(' ');
}

/**
 * Main function to convert English text to Roman Urdu
 * @param text The English text to convert
 * @returns The Roman Urdu text
 */
export function englishToRomanUrdu(text: string): string {
  if (!text || typeof text !== 'string') {
    return '';
  }

  // Use the transliteration function for Roman Urdu conversion
  return transliterateToRomanUrdu(text);
}

/**
 * Advanced transliteration using phonetic patterns
 * This function handles common English-to-Urdu phonetic conversions
 * @param text The English text to transliterate
 * @returns The Roman Urdu transliteration
 */
export function advancedTransliterateToRomanUrdu(text: string): string {
  if (!text || typeof text !== 'string') {
    return '';
  }

  // Convert to lowercase for processing
  let result = text.toLowerCase();

  // Common phonetic patterns in English that have Urdu equivalents
  const phoneticPatterns: [string, string][] = [
    // Common consonant clusters
    ['th', 'th'],  // Aspirated sounds
    ['ch', 'ch'],  // Ch sound
    ['sh', 'sh'],  // Sh sound
    ['ph', 'f'],   // F sound
    ['kh', 'kh'],  // Aspirated sound
    ['gh', 'gh'],  // Gh sound
    ['ng', 'ng'],  // Nasal sound
    ['tr', 'tr'],  // Trill
    ['dr', 'dr'],  // Dr sound
    ['str', 'str'], // Str sound
    ['spl', 'spl'], // Spl sound
    ['spr', 'spr'], // Spr sound
    ['skr', 'skr'], // Skr sound
    ['skl', 'skl'], // Skl sound
    ['st', 'st'],   // St sound
    ['sk', 'sk'],   // Sk sound
    ['sp', 'sp'],   // Sp sound
    ['sl', 'sl'],   // Sl sound
    ['sm', 'sm'],   // Sm sound
    ['sn', 'sn'],   // Sn sound

    // Vowel combinations
    ['ai', 'ai'],
    ['au', 'au'],
    ['ei', 'ei'],
    ['ou', 'ou'],
    ['oa', 'oa'],
    ['oe', 'oe'],
    ['oo', 'oo'],
    ['ee', 'ee'],
    ['ea', 'ea'],
    ['ie', 'ie'],
    ['ue', 'ue'],
    ['ae', 'ae'],
    ['oe', 'oe'],
    ['ue', 'ue'],
  ];

  // Apply phonetic patterns
  for (const [pattern, replacement] of phoneticPatterns) {
    result = result.replace(new RegExp(pattern, 'g'), replacement);
  }

  // Apply common word mappings
  const words = result.split(/\s+/);
  const processedWords: string[] = [];

  for (const word of words) {
    // Check if the entire word exists in the common words map
    if (COMMON_WORDS_MAP.hasOwnProperty(word)) {
      processedWords.push(COMMON_WORDS_MAP[word]);
    } else {
      processedWords.push(word);
    }
  }

  return processedWords.join(' ');
}

/**
 * Main function to convert English text to Roman Urdu
 * @param englishText The English text to convert
 * @returns The Roman Urdu text
 */
export function englishToRomanUrdu(englishText: string): string {
  if (!englishText || typeof englishText !== 'string') {
    return '';
  }

  // Use the advanced transliteration for better results
  return advancedTransliterateToRomanUrdu(englishText);
}

// Export default function for easy usage
export default englishToRomanUrdu;