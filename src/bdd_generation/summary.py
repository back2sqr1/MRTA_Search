#!/usr/bin/python
import pickle

def mostfreq(l):
    c = [(-l.count(x),x) for x in set(l)]
    c.sort()
    return (c[0][1], -c[0][0])

def secondmostfreq(l):
    c = [(-l.count(x),x) for x in set(l)]
    c.sort()
    return (c[1][1], -c[1][0])


all_data = pickle.load(open("pickles/timing-data.pkl", 'rb'))

# plotted=timing-data.pk   0 1 2 _3_ 4 5 6
# timing-data.pk   0 1 1b _2_ 2b 3 4
v = [x['after_cost'] for x in all_data['tate-ex2.srql:[c]']]
print(v)
print("%f: %d of %d = %6.2f per cent." % (min(v), sum([1 for x in v if x == min(v)]), len(v), (sum([1 for x in v if x == min(v)]) * 100.0 / len(v))))
mf = mostfreq(v)
sf = secondmostfreq(v)
print("%f is most frequent (%d times), %f is second most frequent (%d times), that is %f is %f-times more frequent than %f" % (mf[0], mf[1], sf[0], sf[1], mf[0], mf[1]/sf[1], sf[0]))
print("")

# plotted=timing-data.pk   0 1 2 3 _4_ 5 6
# timing-data.pk   0 1 1b 2 _2b_ 3 4
v = [x['after_cost'] for x in all_data['tate-ex2b.srql:[c]']]
print(v)
print("%f: %d of %d = %6.2f per cent." % (min(v), sum([1 for x in v if x == min(v)]), len(v), (sum([1 for x in v if x == min(v)]) * 100.0 / len(v))))
mf = mostfreq(v)
sf = secondmostfreq(v)
print("%f is most frequent (%d times), %f is second most frequent (%d times), that is %f is %f-times more frequent than %f" % (mf[0], mf[1], sf[0], sf[1], mf[0], mf[1]/sf[1], sf[0]))
