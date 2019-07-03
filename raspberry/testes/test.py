import re

text = 'gfgfdAAA1234ZZZuijjk2872389723'

try:
    found = re.findall('[0-9]+', text)
except AttributeError:
    # AAA, ZZZ not found in the original string
    found = '' # apply your error handling

print(found)
# found: 1234
