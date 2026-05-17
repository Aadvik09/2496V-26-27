from pathlib import Path
import re
root = Path('.')
# Update header
hdr = root / 'include' / 'derrickPID.h'
s = hdr.read_text()
# turnPID
s = re.sub(r"extern void turnPID\([^;]+;",
           "extern void turnPID(double desiredValue, int topSpeed, int timeout, double errorThreshold = 15, int settleCount = 50, double chainValue = 0);",
           s)
# driveArcL/R
s = re.sub(r"extern void driveArcL\([^;]+;",
           "extern void driveArcL(double theta, double radius, int timeout, int speed, int chainValue = 0, int errorThreshold = 15, int settleCount = 50);",
           s)
s = re.sub(r"extern void driveArcR\([^;]+;",
           "extern void driveArcR(double theta, double radius, int timeout, int speed, int chainValue = 0, int errorThreshold = 15, int settleCount = 50);",
           s)
# drivePIDW -> drivePID_distance
s = re.sub(r"extern void drivePIDW\([^;]+;",
           "extern void drivePID_distance(int desiredValue, int maxSpeed, int timeout, int wallDistanceTarget, int sensorSide, int dec_point, int chainValue = 0, int errorThreshold = 15, int settleCount = 50, int minSpeed = 30, int triggerDist = -1, int triggerSpeed = 0);",
           s)
hdr.write_text(s)
print('Updated header')

# Update derrickPID.cpp signatures
cpp = root / 'src' / 'derrickPID.cpp'
s = cpp.read_text()
# turnPID signature
s = re.sub(r"void turnPID\([^\)]*\)\s*\{",
           "void turnPID(double desiredValue, int topSpeed = 127, int timeout = 5000, double errorThreshold = 15, int settleCount = 50, double chainDelta = 0)\n{",
           s, count=1)
# driveArcL signature
s = re.sub(r"void driveArcL\([^\)]*\)\s*\{",
           "void driveArcL(double theta, double radius, int timeout = 5000, int speed = 100, int chainValue = 0, int errorThreshold = 15, int settleCount = 50)\n{",
           s, count=1)
# driveArcR signature
s = re.sub(r"void driveArcR\([^\)]*\)\s*\{",
           "void driveArcR(double theta, double radius, int timeout = 5000, int speed = 100, int chainValue = 0, int errorThreshold = 15, int settleCount = 50)\n{",
           s, count=1)
# drivePIDW -> drivePID_distance signature and remove wallOff/on params usage
s = re.sub(r"void drivePIDW\([^\)]*\)\s*\{",
           "void drivePID_distance(int desiredValue, int maxSpeed, int timeout = 5000, int wallDistanceTarget = 5000, int sensorSide = 0, int dec_point = -1, int chainValue = 0, int errorThreshold = 15, int settleCount = 50, int minSpeed = 30, int triggerDist = -1, int triggerSpeed = 0)\n{",
           s, count=1)
# remove references to wallOffStart1/onAgain/etc: replace the block that uses them
s = re.sub(r"// =====================\n\s*// WALL ENABLE/DISABLE[\s\S]*?// =====================\n", "// =====================\n// WALL ENABLE/DISABLE (simplified)\n        bool wallEnabled = true;\n\n", s, count=1)
# write back
cpp.write_text(s)
print('Updated derrickPID.cpp')

# Rewrite call sites in src/auton.cpp for drivePIDW -> drivePID_distance
auton = root / 'src' / 'auton.cpp'
s = auton.read_text()
# pattern for 16-arg calls
pattern = re.compile(r"drivePIDW\(\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^,]+?)\s*,\s*([^\)]+?)\s*\)")

def repl(m):
    a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16 = m.groups()
    # mapping: desiredValue, maxSpeed, timeout, wallDistanceTarget, sensorSide (old index 11), dec_point (12), chainValue (14), errorThreshold (9), settleCount (10), minSpeed (13), triggerDist(15), triggerSpeed(16)
    return f"drivePID_distance({a1}, {a2}, {a3}, {a4}, {a11}, {a12}, {a14}, {a9}, {a10}, {a13}, {a15}, {a16})"

s, n = pattern.subn(repl, s)
print('Rewrote', n, 'drivePIDW calls in auton.cpp')
auton.write_text(s)

# Also update any other files referencing drivePIDW
for path in root.rglob('*.cpp'):
    if path == cpp or path == auton:
        continue
    text = path.read_text()
    if 'drivePIDW(' in text:
        text = text.replace('drivePIDW(', 'drivePID_distance(')
        path.write_text(text)
        print('Rewrote drivePIDW in', path)

print('Done')
