with open("data.txt") as f:
    data = f.read()

num = 10
data = data.split('\n')
del data[-1]
print(data)

xs = data[0].split(' ')
ys = data[1].split(' ')

meanX = 0
for x in xs:
 meanX += float(x)
meanX /= num

meanY = 0
for y in ys:
 meanY += float(y)
meanY /= num

sumTL = 0
sumBR = 0
sumTR = 0

for index in range(num):
   sumTL += ((float(xs[index]) - meanX)**2)
   sumBR += ((float(ys[index]) - meanY)**2)
   sumTR += (float(xs[index]) - meanX) * (float(ys[index]) - meanY)

sumBR /= num
sumTL /= num
sumTR /= num

print(sumTL),
print(sumTR)
print(sumTR),
print(sumBR)
#x = [row.split('\t')[0] for row in data]


