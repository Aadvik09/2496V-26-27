from pathlib import Path
import re
p = Path('src/auton.cpp')
s = p.read_text()
pattern = re.compile(r"(driveArcL|driveArcR)\(\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^\)]+?)\)")

def repl(m):
    fn = m.group(1)
    a1,a2,a3,a4,a5,a6,a7 = m.group(2,3,4,5,6,7,8)
    # new order: target, radius, timeout, max speed, chain value, error width, time in error
    return f"{fn}({a1}, {a2}, {a3}, {a4}, {a7}, {a5}, {a6})"

s_new, n = pattern.subn(repl, s)
print('Reordered', n, 'arc calls')
p.write_text(s_new)
