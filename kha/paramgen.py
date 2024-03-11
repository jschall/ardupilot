#!/usr/bin/env python
import json
import collections
import os
import sys
from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("generate_file", nargs='?')
parser.add_argument("--config", nargs=1)
parser.add_argument("--vehicle-file", nargs=1)
parser.add_argument("--dump-ignored", nargs=1)
parser.add_argument("--selections",nargs=1)
parser.add_argument("--interactive", action='store_true')
args = parser.parse_args()

configfile = args.config[0] if args.config else os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), "paramconfigs.json")

def parseParamFile(fn):
    with open(fn) as f:
        lines = f.readlines()

    import re
    regex = re.compile(r"^\s*([^\s,]+)[,\s]+([+-]?([0-9]+([.][0-9]*)?|[.][0-9]+))\s*(#.+)?$")

    ret = {}
    for line in lines:
        if not line.strip():
            continue
        match = regex.match(line)
        if not match:
            if not line.strip().startswith("#"):
                print("line in file %s did not match:\n%s" % (fn,line))
        else:
            ret[match.group(1)] = "%0.6f" % (float(match.group(2)),)
    return ret

def format_params(paramdict):
    ret = ""
    for name in sorted(paramdict.keys()):
        val = "%0.6f" % (paramdict[name],)
        ret += "%s%s\n" % (name.ljust(17), val.rjust(20))
    return ret

with open(configfile) as f:
    conf = json.JSONDecoder(object_pairs_hook=collections.OrderedDict).decode(f.read())

ignore = set(conf["ignore"])

if args.dump_ignored:
    vehicle_params = parseParamFile(args.dump_ignored[0])
    vehicle_names = set(vehicle_params.keys())

    names = vehicle_names & ignore
    for name in sorted(names):
        print("%s %s" % (name.ljust(17), vehicle_params[name].rjust(20)))
    sys.exit(0)

params = dict(conf["base"])

if args.selections is not None:
    argselections = args.selections[0].split(",")
else:
    argselections = []

for option,selections in conf["options"].items():
    argsel = argselections[0] if argselections else None
    argselections = argselections[1:] if argselections else argselections
    while True:
        if not argsel:
            print ("type an option for %s (%s)" % (option,"|".join(selections.keys())))
        sel = argsel or input()
        argsel = ""
        if sel in selections.keys():
            params.update(dict(selections[sel]))
            break
        else:
            print (sel, "is not a valid choice")

if args.vehicle_file is not None:
    params_rounded = {}
    for name in params:
        params_rounded[name] = "%0.6f" % (params[name])
    vehicle_params = parseParamFile(args.vehicle_file[0])

    vehicle_names = set(vehicle_params.keys())
    config_names = set(params.keys())

    names = (vehicle_names | config_names) - ignore

    if args.interactive:
        for name in sorted(names):
            val1 = params_rounded.get(name,"")
            val2 = vehicle_params.get(name,"")
            if val1 != val2:
                print ("%s%s%s" % ("Name".ljust(16), "In Generated".rjust(20), "On Vehicle".rjust(20)))
                print ("%s%s%s" % (name.ljust(16), val1.rjust(20), val2.rjust(20)))
                print("Press enter to keep generated value, press v to omit from param file")
                res = input()
                if res.strip() == "v":
                    if name in params:
                        del params[name]
                        print("Removed",name)
                
    else:
        print ("### Difference between generated file and provided vehicle file:\n%s%s%s" % ("Name".ljust(16), "In Generated Params".rjust(20), "On Vehicle".rjust(20)))
        for name in sorted(names):
            val1 = params_rounded.get(name,"")
            val2 = vehicle_params.get(name,"")
            if val1 != val2 or not val1:
                print ("%s%s%s" % (name.ljust(16), val1.rjust(20), val2.rjust(20)))
    

if args.generate_file:
    with open(args.generate_file,"w") as f:
        f.write(format_params(params))


if args.generate_file is None and args.vehicle_file is None and args.dump_ignored is None:
    print(format_params(params))
