#!/usr/bin/python
#===============================================================================
#
# conversion script to create a mbstestlib readable file containing test specifications
# out of an testset file in XML format
#
#===============================================================================
# Input can be given via optional command line parameters.
#
#

# TODO: add check for joint count
# TODO: add model description to output (as comment)

import sys # for io
import xml.dom.minidom # for xml parsing
from glob import glob # for expanding wildcards in cmd line arguements

class _config:
    default_input_file = 'testset-example.xml'
    output_file_ext = '.txt'
    
    empty_vecn = ""
    zero_vec = "0 0 0"
    unity_mat = "1 0 0 0 1 0 0 0 1"
    case_defaults = { 'delta': "0.001",
                      'base_r': zero_vec,
                      'base_R': unity_mat,
                      'base_v': zero_vec,
                      'base_omega': zero_vec,
                      'base_vdot': zero_vec,
                      'base_omegadot': zero_vec,
                      'gravitiy': zero_vec,
                      'joints_q': empty_vecn,
                      'joints_qdot': empty_vecn,
                      'joints_qdotdot': empty_vecn,
                      'joints_tau': empty_vecn,
                      'tcp_r': zero_vec,
                      'tcp_R': unity_mat,
                      'tcp_v': zero_vec,
                      'tcp_omega': zero_vec,
                      'tcp_vdot': zero_vec,
                      'tcp_omegadot': zero_vec,
                      'f_ext': zero_vec,
                      'n_ext': zero_vec
                      }
    case_output_order = [
                         'delta',
                         'base_r',
                         'base_R',
                         'base_v',
                         'base_omega',
                         'base_vdot',
                         'base_omegadot',
                         'gravitiy',
                         'joints_q',
                         'joints_qdot',
                         'joints_qdotdot',
                         'joints_tau',
                         'tcp_r',
                         'tcp_R',
                         'tcp_v',
                         'tcp_omega',
                         'tcp_vdot',
                         'tcp_omegadot',
                         'f_ext',
                         'n_ext'
                      ]

class _state:
    error_occured_while_processing_xml = False
    input_file = ''

def getText(nodelist):
    # str(method.childNodes[0].nodeValue) # TODO: remove
    rc = []
    for node in nodelist:
        if node.nodeType == node.TEXT_NODE:
            rc.append(node.data)
    return ''.join(rc)

# inspired by http://code.activestate.com/recipes/52306-to-sort-a-dictionary/
def sortedDict(adict):
    return [ adict[k] for k in sorted(adict.keys()) ]

# parses a specific node and either stores it's value in a dict or the default value
# may set the error bit  
def parse_opt(nodename, valuetype, current_case, current_case_value_dict):
    # if the node does not exist use the default value
    nodelist = current_case.getElementsByTagName(nodename)
    if nodelist.length == 0:
        current_case_value_dict.update({nodename : _config.case_defaults.get(nodename)})
    elif nodelist.length > 1:
        _state.error_occured_while_processing_xml = True
        print("'" + nodename + "' defined more than once.")
        return
    else: 
        # we have one single node to parse
        node = nodelist[0]
        value = node.getAttribute(valuetype)
        if value == None:
            # TODO: more advanced checks with regexp
            _state.error_occured_while_processing_xml = True
            print("'" + nodename + "' has an empty value or wrong type ('"+ valuetype +"').")
            return
        else :
            current_case_value_dict.update({nodename : value})
            return

def convert_xml_testset_2_raw_testset(mbs_test_set):
    raw_testsets = dict([]) # filename:content dict
    
    for mbs in mbs_test_set.getElementsByTagName('mbs'): # for every file
        file = mbs.getAttribute('file')
        raw_testset = []
        
        if mbs.getElementsByTagName('model').length != 1:
            _state.error_occured_while_processing_xml = True
            print("Only one model allowed per file!")
            return dict([])
        
        # extract model
        raw_testset.append("% " + mbs.getElementsByTagName('model')[0].getAttribute('desc'))
        raw_testset.append(getText(mbs.getElementsByTagName('model')[0].childNodes))

        # insert separation marker
        raw_testset.append("\nendmodel")

        # now process the cases
        if mbs.getElementsByTagName('case').length == 0:
            _state.error_occured_while_processing_xml = True
            print("No cases defined!")
            return dict([])
        cases = dict([])
        for case in mbs.getElementsByTagName('case'):
            # TODO: sanity check -> number collisions
            # parse case
            case_nr = case.getAttribute('nr')
            case_desc = case.getAttribute('desc')
            case_value_dict = dict([])
            # everything but joints does not have to be defined explicitly
            # TODO: unify these calls in a generic way (e.g. add type to case_output_order and iterate over it) 
            parse_opt('delta', 'scalar', case, case_value_dict)
            parse_opt('base_r', 'vector3', case, case_value_dict)
            parse_opt('base_R', 'matrix3x3', case, case_value_dict)
            parse_opt('base_v', 'vector3', case, case_value_dict)
            parse_opt('base_omega', 'vector3', case, case_value_dict)
            parse_opt('base_vdot', 'vector3', case, case_value_dict)
            parse_opt('base_omegadot', 'vector3', case, case_value_dict)            
            parse_opt('gravitiy', 'vector3', case, case_value_dict)
            # TODO: checks with n (the number of joints)
            parse_opt('joints_q', 'vector_n', case, case_value_dict)              
            parse_opt('joints_qdot', 'vector_n', case, case_value_dict)
            parse_opt('joints_qdotdot', 'vector_n', case, case_value_dict)
            parse_opt('joints_tau', 'vector_n', case, case_value_dict)
            parse_opt('tcp_r', 'vector3', case, case_value_dict)
            parse_opt('tcp_R', 'matrix3x3', case, case_value_dict)
            parse_opt('tcp_v', 'vector3', case, case_value_dict)
            parse_opt('tcp_omega', 'vector3', case, case_value_dict)
            parse_opt('tcp_vdot', 'vector3', case, case_value_dict)
            parse_opt('tcp_omegadot', 'vector3', case, case_value_dict)                 
            parse_opt('f_ext', 'vector3', case, case_value_dict) 
            parse_opt('n_ext', 'vector3', case, case_value_dict) 
            if _state.error_occured_while_processing_xml: return dict([])
            
            # compile raw case output
            case_content = ["\n" + case_desc]
            for value_name in _config.case_output_order:
                if case_value_dict.get(value_name) is None :
                    _state.error_occured_while_processing_xml = True
                    print("Not all values defined in one testcase!")
                    return dict([])
                case_content.append(case_value_dict.get(value_name))
            cases.update({case_nr : "\n".join(case_content)})
        # flatten cases (and sort)
        raw_testset.append("\n".join(sortedDict(cases)))
        # update file:testset dict
        raw_testsets.update({file : "\n".join(raw_testset)})
    # return the dict of files:testsets    
    return raw_testsets



#===============================================================================
# process command line arguments (i.e. file i/o)
#===============================================================================
script_name = sys.argv[0][sys.argv[0].rfind("\\")+1:]
if len(sys.argv) == 1:
    _state.input_file = _config.default_input_file
    print("No command line arguments were given. Defaulting to:")
    print("Input '" + _state.input_file + "'")
    print("Usage hint: " + script_name + " [INPUTFILE(s)]\n")
elif len(sys.argv) == 2:
    if sys.argv[1] == "--help":
        print("Usage: " + script_name + " [INPUTFILE(s)]")
        sys.exit()
    else:
        _state.input_file = glob(sys.argv[1])

#===============================================================================
# run the conversion
#===============================================================================
for inputfile in _state.input_file :
    xmldom = xml.dom.minidom.parse(inputfile)
    raw_testsets = convert_xml_testset_2_raw_testset(xmldom.firstChild)
    if not _state.error_occured_while_processing_xml :
        for k in raw_testsets.keys():
            with open(k, 'w') as raw_testset_file:
                raw_testset_file.write(raw_testsets.get(k))
                print("File '" + k + "' written.")

#===============================================================================
# concluding housekeeping 
#===============================================================================
if not _state.error_occured_while_processing_xml:
    print("Conversion successful.")
else:
    print("The xml file could not be processed properly. It most likely contains errors.")
sys.exit(_state.error_occured_while_processing_xml)
