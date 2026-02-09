#!/usr/bin/python
import subprocess
import sys
import pickle
import statistics

collect_more_data = True # Otherwise just load the data

#reps = 10  # Number of times to run with same initial ordering
#runs = 50 # Number of initial random orderings to try
#inputs = ["examples/ex1.sqrl", "examples/ex2.sqrl", "examples/ex3.sqrl", "examples/ex4.sqrl", "examples/ex5.sqrl"]
#inputs_names = ["example1", "example2", "example3", "example4", "example5"] # pretty names
#meths = ["s", "c", "[c]", "[c][s]", "[s][c]",]

reps = 1  # Number of times to run with same initial ordering (there is no non-determinism in the output, only running times)
runs = 100 # Number of initial random orderings to try
inputs = [ "examples/tate-ex5.srql", "examples/tate-ex6.srql"]
#inputs_names = ["Any Marilyn Diptych?", "Marilyn Diptych at 15", "Marilyn Diptych at 15 (alt. start)", "Marilyn Diptych in an L?", "Marilyn Diptych in an L? (alt. obs costs)", "Yinka Shonibare?", "Yinka Shonibare and (Marilyn Diptych at 15)?"]
inputs_names = ["Query 0", "Query 1", "Query 1b", "Query 2", "Query 2b", "Query 3", "Query 4"]
inputs_names_sz = 8
#meths = ["s", "c", "[c]",  "[c][s]", "[s][c]"]
meths = ["c"]
meth_names = {"s": "Rudell's classic sifting algorithm",  # pretty names
              "c": "Rudell's sifting with plan costs", 
              "[c]": "Sifting-by-block with plan costs", 
              "[c][s]": "Block sifted for plan costs the size", 
              "[s][c]": "Block sifted for size then plan costs", 
              #"[s][c]": "Blocks-sifted for size then Sifting-by-block with Plan Costs", 
              #"[c][s]": "Sifting-by-block with Plan Costs followed by blocks-sifted for size", 
              }
#meth_cols = {"s":"C2", "c":"C8", "[c]":"C0", "[c][s]":"C6", "[s][c]":"C4",}
meth_cols = {"s":"C0", "c":"C6", "[c]":"C1", "[c][s]":"C2", "[s][c]":"C3",}

picklefile = "pickles/timing-data.pkl"

try:
    all_data = pickle.load(open(picklefile, 'rb'))
except:
    all_data = {}

def key_for_experiment(ifile, imeth):
    return ifile.split('/')[-1] + ":" + str(imeth) 

try:
    t = all_data[key_for_experiment(inputs[0], meths[0])]
except: 
    # There is no data, so we must collect
    print("There appears to be no data in the pickle file, so we have to collect some")
    collect_more_data = True


if collect_more_data:
    for r in range(runs):
        for inp in inputs:
            for m in meths:
                    sd = r
                    k = key_for_experiment(inp, m)
                    cmd = f'./srql-planner.py'
                    cargs = [f'--method={m}', f'--reps={reps}', f'--seed={sd}' ,f'{inp}']
                    #print([cmd]+cargs)
                    o = subprocess.run([cmd]+cargs, capture_output=True)
                    if o.returncode != 0:
                        print("Process returned failure code!?! Command was: %s" % (" ".join([cmd]+cargs)))
                    else:
                        out = (o.stdout)
                        d = eval(out)

                        if (len(d["times"]) > 1):
                            print("Process time elapsed: μ =%6.2f seconds, σ =%6.2f seconds" % (statistics.mean(d["times"]), statistics.stdev(d["times"])))
                        else:
                            print("Process time elapsed: %6.2f seconds" % (statistics.mean(d["times"])))
                        #print("Process time elapsed: μ =%6.2f seconds, σ =%6.2f seconds" % (statistics.mean(d["times"]), statistics.stdev(d["times"])))
                        #print("Size changed from %d to %d" % (d["before_size"], d["after_size"]))
                        #print("Cost changed from %d to %d" % (d["before_cost"], d["after_cost"]))

                        try:
                            prior = all_data[k]
                        except:
                            prior = []
                        prior.append(d)
                        all_data[k] = prior
        print("Saving data")
        pickle.dump(all_data, open(picklefile, "wb"))


# Try to plot the data we have:
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
import numpy as np
import random 

meth_offsets = {}
width = 0.11  # the width of the bars
step = 1.1
off = -.5 * (len(meths) * step)
for m in meths:
    off = off + step/2.0
    meth_offsets[m] = off*width
    off = off + step/2.0

sc_pos = []
sc_val = []
ave_times = {}
std_times = {}
for m in meths:
    aves = []
    stds = []
    xpos = 0
    for inp in inputs:
        k = key_for_experiment(inp, m)
        times = []
        for i in all_data[k]:
            times = times + i['times']
        aves.append(statistics.mean(times))
        stds.append(statistics.stdev(times))
        for t in times:
            sc_pos.append(xpos+meth_offsets[m])
            sc_val.append(t)
        xpos = xpos + 1


    ave_times[m] = aves
    std_times[m] = stds


x = np.arange(len(inputs))  # the label locations

maxrange = 6.5

fig, (ax, ax2) = plt.subplots(2,1, sharex=True, figsize=(6.4, 4), dpi=80, gridspec_kw={'height_ratios': [1, 4]})
fig.subplots_adjust(hspace=0.05)  # adjust space between axes


plt.gcf().subplots_adjust(bottom=0.15)
#plt.scatter(sc_pos, sc_val, s = 0.25, c ='#afafaf', zorder = 10)
for m in meths:
    bars1 = ax.bar(x + meth_offsets[m], ave_times[m], width, yerr=std_times[m], label=meth_names[m], ecolor='black', capsize=3, zorder=2, color=meth_cols[m])
    bars2 = ax2.bar(x + meth_offsets[m], ave_times[m], width, yerr=std_times[m], label=meth_names[m], ecolor='black', capsize=3, zorder=2, color=meth_cols[m])

    #for i in range(len(ave_times[m])):
    #    t = ave_times[m][i]
    #    if  t > maxrange:
    #        l = (f" %6.1f" % t)
    #        ax.text(i-0.165, maxrange-maxrange/10.0, l, color = 'black', size=6, rotation=90)


# Add some text for labels, title and custom x-axis tick labels, etc.



plt.xticks(fontsize=inputs_names_sz, rotation=00)
ax.set_xticks(x) 
ax.set_xticklabels(inputs_names)
ax2.legend(loc="upper left", bbox_to_anchor=(0.00,0.975), prop={'size': 7})
ax.set_title('Computation time per method vs. Problem instance', fontweight ='bold', size=12)
ax.set_ylim(10,230)
ax2.set_ylim(0, maxrange)

ax.spines['bottom'].set_visible(False)
ax2.spines['top'].set_visible(False)
ax.xaxis.tick_top()
ax.tick_params(labeltop=False)
ax2.xaxis.tick_bottom()

# Put in the diagonal line
d = .5  # proportion of vertical to horizontal extent of the slanted line
kwargs = dict(marker=[(-1, -d), (1, d)], markersize=12,
              linestyle="none", color='k', mec='k', mew=1, clip_on=False)
ax.plot([0, 1], [0, 0], transform=ax.transAxes, **kwargs)
ax2.plot([0, 1], [1, 1], transform=ax2.transAxes, **kwargs)


plt.xlabel("Problem instance", fontweight ='bold', size=10)
plt.ylabel("Computation time (seconds)", fontweight ='bold', size=10)
plt.savefig('instances-vs-time.pdf', format='pdf')
plt.close(plt.gcf())




if False:
    fig, ax = plt.subplots(figsize=(4, 4), dpi=80)
    #ax.arrow(0, 0, 0.5, 0.5, head_width=0.05, head_length=0.1, fc='k', ec='k')

    minx = miny = 10000
    maxx = maxy = -1
    for inp in inputs[2:3]:
        for m in meths:
            k = key_for_experiment(inp, m)
            for i in all_data[k]:
                x1 = i['before_size']
                x2 = i['after_size']
                y1 = i['before_cost']
                y2 = i['after_cost']
                ax.arrow(x1, y1, x2-x1, y2-y1, head_width=0.05, head_length=0.1, fc='r', ec=meth_cols[m])
                minx = min(minx,x1,x2)
                maxx = max(maxx,x1,x2)
                miny = min(miny,y1,y2)
                maxy = max(maxy,y1,y2)

    plt.xlim([minx-1, maxx+1])
    plt.ylim([miny-1, maxy+1])
    plt.xlabel("Size", fontweight ='bold', size=10)
    plt.ylabel("Plan cost", fontweight ='bold', size=10)

    plt.savefig('quality.pdf', format='pdf')
    plt.close(plt.gcf())


if False:
    fig1, ax1 = plt.subplots(figsize=(6.4, 3), dpi=80)
    plt.gcf().subplots_adjust(bottom=0.15)

    for m in meths:
        xpos = 0
        for inp in inputs:
            s_pos = []
            s_val = []
            k = key_for_experiment(inp, m)
            for i in all_data[k]:
                s_pos.append(xpos+meth_offsets[m])# +0.02*random.random()) # Visual jitter
                s_val.append(i['after_size'])
            midx = xpos + meth_offsets[m]
            midy = statistics.mean(s_val)
            stdd = statistics.stdev(s_val)
            ax1.errorbar(midx, midy, yerr = stdd, xerr = None, linewidth=1.0, markersize = 1.2, marker='o', ls='none', capsize=2, c=meth_cols[m], alpha=1.00) 
            xpos = xpos + 1
            #ax1.scatter(s_pos, s_val, c=meth_cols[m], s = 8, linewidth=0.5, marker='+', zorder = 10, alpha=0.75)

    for m in meths: # just put in values suitable for the legend
        ax1.bar(x + meth_offsets[m], 0*x, width, label=meth_names[m], zorder = 2, color=meth_cols[m])


    ax1.set_xticks(x)
    ax1.set_xticklabels(inputs_names)
    plt.xticks(fontsize=inputs_names_sz, rotation=00)
    ax1.legend(loc=2, prop={'size': 7})
    ax1.set_title('Final BDD size vs. Problem instance', fontweight ='bold', size=12)
    plt.xlabel("Problem instance", fontweight ='bold', size=10)
    ax1.set_ylabel("Size (vertices)")
    ax1.set_ylim([0, 525])
    #ax2.set_ylabel("Costs", color='blue')
    #ax1.ylabel("Size (vertices)", fontweight ='bold', size=10)
    #ax1.ylim([0, 125])
    #ax2.ylabel("Costs", fontweight ='bold', size=10)
    #ax2.ylim([0, 50])
    plt.savefig('quality-size.pdf', format='pdf')
    plt.close(plt.gcf())


if False:
    fig1, ax1 = plt.subplots(figsize=(6.4, 3), dpi=80)
    plt.gcf().subplots_adjust(bottom=0.15)

    for m in meths:
        xpos = 0
        for inp in inputs:
            c_pos = []
            c_val = []
            k = key_for_experiment(inp, m)
            for i in all_data[k]:
                c_pos.append(xpos+meth_offsets[m])# +0.02*random.random()) # Random jitter to move the points
                c_val.append(i['after_cost'])
            midx = xpos + meth_offsets[m]
            midy = statistics.mean(c_val)
            stdd = statistics.stdev(c_val)
            ax1.errorbar(midx, midy, yerr = stdd, xerr = None, linewidth=1.0, markersize = 1.2, marker='o', ls='none', capsize=2, c=meth_cols[m], alpha=1.00) 
            xpos = xpos + 1
            #ax1.scatter(c_pos, c_val, c=meth_cols[m], s = 6, linewidth=0.5, marker='x', zorder = 3, alpha=0.75)

    for m in meths: # just put in values suitable for the legend
        ax1.bar(x + meth_offsets[m], 0*x, width, label=meth_names[m], zorder = 2, color=meth_cols[m])

    ax1.set_xticks(x)
    ax1.set_xticklabels(inputs_names)
    plt.xticks(fontsize=inputs_names_sz, rotation=00)
    ax1.legend(loc=2, prop={'size': 7})
    ax1.set_title('Plan Cost vs. Problem instance', fontweight ='bold', size=12)
    plt.xlabel("Problem instance", fontweight ='bold', size=10)
    ax1.set_ylabel("Execution cost")
    #ax1.set_ylim([475, 480])
    plt.savefig('quality-cost.pdf', format='pdf')
    plt.close(plt.gcf())


fig1, ax1 = plt.subplots(figsize=(6.4, 4.5), dpi=80)
plt.gcf().subplots_adjust(bottom=0.15)
gs = gridspec.GridSpec(2, 1, height_ratios=[1.2, 2])
ax1 = plt.subplot(gs[0])
ax2 = plt.subplot(gs[1], sharex = ax1)


for m in meths:
    xpos = 0
    for inp in inputs:
        s_pos = []
        s_val = []
        c_pos = []
        c_val = []
        k = key_for_experiment(inp, m)
        for i in all_data[k]:
            s_pos.append(xpos+meth_offsets[m])# +0.02*random.random()) # Visual jitter
            s_val.append(i['after_size'])
            c_pos.append(xpos+meth_offsets[m])# +0.02*random.random()) # Random jitter to move the points
            c_val.append(i['after_cost'])
        midx = xpos + meth_offsets[m]
        midy = statistics.mean(c_val)
        stdd = statistics.stdev(c_val)
        ax2.errorbar(midx, midy, yerr = stdd, xerr = None, linewidth=2.5, markersize = 3.6, marker='o', ls='none', capsize=3.4, c=meth_cols[m], alpha=1.00) 

        midy = statistics.mean(s_val)
        stdd = statistics.stdev(s_val)
        ax1.errorbar(midx, midy, yerr = stdd, xerr = None, linewidth=2.5, markersize = 3.6, marker='o', ls='none', capsize=3.4, c=meth_cols[m], alpha=1.00) 

        xpos = xpos + 1
        #ax1.scatter(s_pos, s_val, c=meth_cols[m], s = 8, linewidth=0.5, marker='+', zorder = 10, alpha=0.75)
        #ax1.scatter(c_pos, c_val, c=meth_cols[m], s = 6, linewidth=0.5, marker='x', zorder = 3, alpha=0.75)

for m in meths: # just put in values suitable for the legend
    ax2.bar(x + meth_offsets[m], 0*x, width, label=meth_names[m], zorder = 2, color=meth_cols[m])

# remove last tick label for the second subplot

ax1.set_xticks(x)
ax1.set_xticklabels(inputs_names)
plt.xticks(fontsize=inputs_names_sz, rotation=00)
ax2.legend(loc="upper left", bbox_to_anchor=(0.0,1.10),  prop={'size': 7}, framealpha=1.0, ncol=1, columnspacing=0.8)
ax1.set_title('Resulting plan vs. Problem instance', fontweight ='bold', size=12)
plt.xlabel("Problem instance", fontweight ='bold', size=10)
ax1.set_ylabel("Plan size", fontweight ='bold', size=10)
ax1.set_ylim([1, 540])
#yticks = ax1.yaxis.get_major_ticks()
#yticks[1].label1.set_visible(False)
ax2.set_ylabel("Execution cost", fontweight ='bold', size=10)
ax2.set_ylim([0, 575])
plt.subplots_adjust(hspace=.0)
plt.savefig('quality-combined.pdf', format='pdf')
plt.close(plt.gcf())


