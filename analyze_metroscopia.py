import re
from collections import Counter
from pathlib import Path
path = Path(r'c:\Work\metroscope-modeling-library\MetroscopeModelingLibrary\Examples\Nuclear\MetroscopiaNPP\MetroscopiaNPP_direct.mo')
text = path.read_text(encoding='utf-8')
# extract equation block text
m = re.search(r'equation\n(.*)annotation\s*\(', text, re.S)
block = m.group(1) if m else ''
assigns = []
for line in block.splitlines():
    s = line.strip()
    if not s or s.startswith('//') or s.startswith('connect(') or s.startswith('annotation'):
        continue
    # ignore interior annotation continuation lines
    if s.startswith('{') or s.startswith('color=') or s.startswith(')'):
        continue
    m2 = re.match(r'([A-Za-z_][A-Za-z0-9_\.]*?)\s*=\s*[^;]*;', s)
    if m2:
        assigns.append(m2.group(1))
print('assignments', len(assigns))
for name, count in Counter(assigns).most_common():
    if count > 1:
        print('dup', count, name)
print('---')
print('\n'.join(assigns))
