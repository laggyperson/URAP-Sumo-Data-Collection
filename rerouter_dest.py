"""
Script to generate rerouters that go to random destination edges given set of edges in graph and a file to output it to.
Default is the G16 edges and predetermined filename

Author: Phillip Chen
Date: July 2023
"""
"""
Edges for Gomentum: ["16", "-11", "-0", "3", "6", "-24", "13", "15", "12", "4", "22", "25", "5", "21", "14"]
Edges for Town01: ["0", "1", "-1", "2", "-2", "-3", "-4", "4", "5", "6", "-6", "7", "-7", "-8", "-9", "9", "10", "-10", "12", "-12", "16", "-16", "17", "-17", "18", "-18", "-19", "19", "21", "-21", "22", "-22", "23", "-23", "24", "-24", "25", "-25"]
Edges for Town03: ["-0", "0", "-1", "1", "-2", "2","-3", "3", "-4", "4", "-77", "77", "-78", "78", 
                    "-76", "76", "-75", "75", "-74", "74", "-87", "87", "-18", "18", "-19", "19", 
                    "-20", "20", "-21", "21", "-22", "22", "-52", "-23", "23", "-25", "25", "-90", "90", 
                    "-57", "57", "-92", "-56", "56", "-5", "6", "-7", "7", "-8", "8", "-67", "67", 
                    "-68", "68", "-65", "-87", "87", "-88", "88", "-89", "89", "-49", "49", "-50", "50", 
                    "-51", "51", "-31", "31", "-32", "32", "-33", "33", "-34", "34"] (NOTE: 67, 18, 33 dont work with other edges)
"""

import argparse
import xml.etree.ElementTree as ET
import xml.dom.minidom

argparser = argparse.ArgumentParser(description="Optional: Specify edges in the graph to add rerouters for")

argparser.add_argument("--edges", "-e",
                       metavar="E",
                       type=str,
                       nargs="*",
                       dest="edges",
                       default=["-0", "0", "-1", "1", "-2", "2","-3", "3", "-4", "4", "-77", "77", "-78", "78", 
                                "-76", "76", "-75", "75", "-74", "74", "-87", "87", "-19", "19", 
                                "-20", "20", "-21", "21", "-22", "22", "-52", "-23", "23", "-25", "25", "-90", "90", 
                                "-57", "57", "-92", "-56", "56", "-5", "6", "-7", "7", "-8", "8", 
                                "-68", "68", "-65", "-87", "87", "-88", "88", "-89", "89", "-49", "49", "-50", "50", 
                                "-51", "51", "-31", "31", "-32", "32", "-34", "34"],
                       help="List edges, separating with spaces. Default is G16 edges")
argparser.add_argument("--output-file", "-o",
                       metavar="O",
                       type=str,
                       default="add/G16_equal_dest.add.xml",
                       dest="output_file",
                       help="The file to output the rerouters to; default in additional folder for G16.")
args = argparser.parse_args()

edges = args.edges
output_file = args.output_file

prob = round(1 / (len(edges) - 1), 4)

# Take into account that the probabilities have to add up to 1
extra = round(1 - (prob * (len(edges) - 2)), 4)

# The root of the file, which specifies additional type
root = ET.Element("additional")

for i in range(len(edges)):
    rr_id = "rr" + "_" + str(i)
    rerouter = ET.Element("rerouter")
    rerouter.set("id", rr_id)
    rerouter.set("edges", edges[i])
    rerouter.set("pos", "0.0")

    # The time the rerouter is active for
    interval = ET.SubElement(rerouter, "interval")
    interval.set("begin", "0.0")
    interval.set("end", "3600.00")

    for j in range(len(edges)):
        if i != j and j == len(edges) - 1: # Biases towards the last edge just a little bit
            dest = ET.SubElement(interval, "destProbReroute")
            dest.set("id", edges[j])
            dest.set("probability", str(extra))
        elif i != j:
            dest = ET.SubElement(interval, "destProbReroute")
            dest.set("id", edges[j])
            dest.set("probability", str(prob))

    root.append(rerouter)

# Creating xml tree
tree = ET.ElementTree(root)
tree = xml.dom.minidom.parseString(ET.tostring(tree.getroot())).toprettyxml(indent="    ")

# Creating and writing to file
with open(output_file, "w") as files:
    files.write(tree)

print("Successfully created the file.")