"""
Script to generate flows that reach a target number of vehicles, given set of edges in graph and a file to output it to.
Default is the G16 edges and predetermined filename

Author: Phillip Chen
Date: April 2024
"""
"""
Edges for Gomentum: ["16", "-11", "-0", "3", "6", "-24", "13", "15", "12", "4", "22", "25", "5", "21", "14"]
Edges for Town01: ["0", "1", "-1", "2", "-2", "-3", "-4", "4", "5", "6", "-6", "7", "-7", "-8", "-9", "9", 
                    "10", "-10", "12", "-12", "16", "-16", "17", "-17", "18", "-18", "-19", "19", "21", "-21", 
                    "22", "-22", "23", "-23", "24", "-24", "25", "-25"]
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

argparser = argparse.ArgumentParser(description="Specify edges in the graph to add flows for. Specify the number of vehicles desired.")

argparser.add_argument("--edges", "-e",
                       metavar="E",
                       type=str,
                       nargs="*",
                       dest="edges",
                       default=["0", "1", "-1", "2", "-2", "-3", "-4", "4", "5", "6", "-6", "7", "-7", "-8", "-9", "9", 
                    "10", "-10", "12", "-12", "16", "-16", "17", "-17", "18", "-18", "-19", "19", "21", "-21", 
                    "22", "-22", "23", "-23", "24", "-24", "25", "-25"],
                       help="List edges, separating with spaces. Default is G16 edges")
argparser.add_argument("--output-file", "-o",
                       metavar="O",
                       type=str,
                       default="add/G16_equal_dest.add.xml",
                       dest="output_file",
                       help="The file to output the rerouters to; default in additional folder for G16.")
argparser.add_argument("--num-vehicles", "-n",
                       metavar="N",
                       type=int,
                       default=100,
                       dest="num_vehicles",
                       help="Specify the number of edges you want in this simulation.")
args = argparser.parse_args()

edges = args.edges
output_file = args.output_file
num_vehicles = args.num_vehicles

n_edge = len(edges)
# Number of vehicles per edge
n_veh_edge = num_vehicles // n_edge
flow_cnts = {i: n_veh_edge for i in range(n_edge)}
# Round robin the remainder
for i in range(num_vehicles - n_edge * n_veh_edge):
    flow_cnts[i % n_edge] += 1

# The root of the file, which specifies routes type
root = ET.Element("routes")

for i in range(n_edge):
    # The element and its id
    flow_id = "f" + "_" + str(i)
    flow = ET.Element("flow")
    
    # Setting the element parameters
    flow.set("id", flow_id)
    flow.set("type", "DEFAULT_VEHTYPE")
    flow.set("from", edges[i])
    flow.set("to", edges[i])
    flow.set("vehsPerHour", "1000.00")
    flow.set("departLane", "random")
    flow.set("begin", "0.0")
    flow.set("number", str(flow_cnts[i]))

    root.append(flow)

# Creating xml tree
tree = ET.ElementTree(root)
tree = xml.dom.minidom.parseString(ET.tostring(tree.getroot())).toprettyxml(indent="    ")

# Creating and writing to file
with open(output_file, "w") as files:
    files.write(tree)

print("Successfully created the file.")