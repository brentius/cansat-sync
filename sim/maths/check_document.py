"""GPL-3.0-only. Structural checks; not a substitute for TeX compilation."""
from collections import Counter
from pathlib import Path
import json
import re

p=Path(__file__).resolve().parents[2]/'wiki'/'cansat_mathematics.tex'
s=p.read_text(encoding='utf-8')
assert not [ord(c) for c in s if ord(c)<32 and c not in '\n\t'], 'embedded control byte'
assert s.count(r'\begin{document}')==s.count(r'\end{document}')==1
assert not re.search(r'\\(?:input|include|includegraphics|bibliography)\b',s), 'external compile resource'
# Strip comments while preserving escaped percent signs.
lines=[re.split(r'(?<!\\)%',line,maxsplit=1)[0] for line in s.splitlines()]
clean='\n'.join(lines)
braces=0
for match in re.finditer(r'(?<!\\)[{}]',clean):
    braces+=1 if match.group()=='{' else -1
    assert braces>=0, 'closing brace before opening brace'
assert braces==0, 'unbalanced braces'
stack=[]
for m in re.finditer(r'\\(begin|end)\{([^}]+)\}',clean):
    if m[1]=='begin': stack.append(m[2])
    else:
        assert stack and stack[-1]==m[2], f'environment mismatch at {m[2]}'
        stack.pop()
assert not stack
labels=re.findall(r'\\label\{([^}]+)\}',clean)
refs=re.findall(r'\\(?:ref|eqref)\{([^}]+)\}',clean)
assert not [label for label,count in Counter(labels).items() if count>1], 'duplicate labels'
assert not set(refs)-set(labels), 'undefined references'
commands='operatorname|mathrm|mathbf|mathsf|mathbb|frac|widehat|boldsymbol|underbrace|overline|substack'
assert not re.search(r'(?<![A-Za-z\\])(?:'+commands+r')(?=\{)',clean), 'lost command backslash'
report={'source':str(p),'bytes':p.stat().st_size,'lines':len(s.splitlines()),'labels':len(labels),
        'reference_uses':len(refs),'numbered_math_environments':len(re.findall(r'\\begin\{(?:equation|align)\}',clean)),
        'standalone':True,'structural_checks_passed':True,
        'compilation':'UNVERIFIED: built-in compiler fails: Unable to find standard directories for platform',
        'limitation':'Structure and label checks do not verify TeX macro availability, line/page layout or PDF rendering.'}
out=Path(__file__).resolve().parent/'results'/'document_checks.json'
out.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
