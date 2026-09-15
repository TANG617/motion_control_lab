#!/usr/bin/env python3
"""Validate and copy the current public app declaration into a new file.
Reads explicit sources; does not migrate, generate inputs, or execute an app.
"""
import argparse,pathlib,sys
HERE=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/app_execution'))
from contracts import prepare_definition

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--definition',type=pathlib.Path,default=HERE/'definition.json')
    p.add_argument('--output',type=pathlib.Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    prepare_definition(a.definition,a.output)
    print(a.output)
if __name__=='__main__':main()
