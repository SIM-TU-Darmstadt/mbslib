#!/usr/bin/python
#===============================================================================
#
# conversion script to create a human readable table in html out of an xml log file
#
#===============================================================================
# Input and output can be given via optional command line parameters.
#
# assumptions:
#     - each method/algorithm is uniquely named
#

import sys # for io
import xml.dom.minidom # for xml parsing
import cgi #  for escaping html



class _config:
    input_file = 'testlog.xml'
    output_file = 'testlog.html'
   
class _state:
    error_occurence_in_tests = False

class _gathered_infos:
    dirkin_methods = set()
    invdyn_methods = set()
    dirdyn_methods = set()
    hybdyn_methods = set()
    # not used
    #testfiles = [] # pairs of filename and description (one model per file)
       
def gather_infos(mbs_test_protocol):
    for mbs in mbs_test_protocol.getElementsByTagName('mbs'):
        # not used
        #_gathered_infos.testfiles.append((mbs.getAttribute('file'),mbs.getAttribute('desc')))
        for case in mbs.getElementsByTagName('case'):
            for dirkin_test in case.getElementsByTagName('dirkin'):
                _gathered_infos.dirkin_methods.add(dirkin_test.getAttribute('method'))
            for invdyn_test in case.getElementsByTagName('invdyn'):
                _gathered_infos.invdyn_methods.add(invdyn_test.getAttribute('method'))
            for dirdyn_test in case.getElementsByTagName('dirdyn'):
                _gathered_infos.dirdyn_methods.add(dirdyn_test.getAttribute('method'))
            for hybdyn_test in case.getElementsByTagName('hybdyn'):
                _gathered_infos.hybdyn_methods.add(hybdyn_test.getAttribute('method'))


 
def generate_html_table_header_cell(html, header_cell_content, th_attributes=""):
    if len(th_attributes) > 0:
        th_attributes = " " + th_attributes
    html.append("<th%s>%s</th>" % (th_attributes, header_cell_content))
    
def generate_html_table_cell(html, cell_content, td_attributes=""):
    if len(td_attributes) > 0:
        td_attributes = " " + td_attributes
    html.append("<td%s>%s</td>" % (td_attributes, cell_content))
    
def getText(nodelist):
    rc = []
    for node in nodelist:
        if node.nodeType == node.TEXT_NODE:
            rc.append(node.data)
    return ''.join(rc)

 
def convert_xml_2_html_table(mbs_test_protocol):
    gather_infos(mbs_test_protocol)
    methods_list = list(_gathered_infos.dirkin_methods) + list(_gathered_infos.invdyn_methods) + list(_gathered_infos.dirdyn_methods) +  list(_gathered_infos.hybdyn_methods) 

    html = []

    def table_row_open():
        html.append("<tr>")
    def table_row_close():
        html.append("</tr>")
    
    #==========================================================================
    # create the table header 
    #==========================================================================
    table_row_open()
    generate_html_table_header_cell(html, "&nbsp;") # filename field
    generate_html_table_header_cell(html, "case #")
    generate_html_table_header_cell(html, "direct kinematics", "colspan=\"" + str(len(_gathered_infos.dirkin_methods)) + "\"")
    generate_html_table_header_cell(html, "inverse dynamics", "colspan=\"" + str(len(_gathered_infos.invdyn_methods)) + "\"")
    generate_html_table_header_cell(html, "direct dynamics", "colspan=\"" + str(len(_gathered_infos.dirdyn_methods)) + "\"")
    generate_html_table_header_cell(html, "hybrid dynamics", "colspan=\"" + str(len(_gathered_infos.hybdyn_methods)) + "\"")
    table_row_close()
    table_row_open()
    for header in ["&nbsp;", "&nbsp;"] + methods_list:
        generate_html_table_header_cell(html, header)
    table_row_close()

    # table body
    for mbs in mbs_test_protocol.getElementsByTagName('mbs'):
        #=======================================================================
        # handle one testfile (i.e. one row for every case in the testfile)
        #=======================================================================
        filecell = "<a title=\"" + cgi.escape(mbs.getAttribute('desc')) + "\">" + cgi.escape(mbs.getAttribute('file')) + "</a>"
        filename_cell_already_created = False
        for case in mbs.getElementsByTagName('case'):
            table_row_open()
            if not filename_cell_already_created:
                # row description (testfile wide) (html standard requires the first row to be included in this one
                generate_html_table_cell(html, filecell, "rowspan=\"" + str(len(mbs.getElementsByTagName('case'))) + "\"")
                filename_cell_already_created = True 
            # row description (case part)
            generate_html_table_cell(html, "<a title=\"" + cgi.escape(case.getAttribute('desc')) + "\">" + case.getAttribute('nr') + "</a>")
            
            #===================================================================
            # process (one) case's actual results
            #===================================================================
            result_list = ["-" for i in range(len(methods_list))]
            for method in case.getElementsByTagName('dirkin') + case.getElementsByTagName('invdyn') + case.getElementsByTagName('dirdyn') + case.getElementsByTagName('hybdyn'):
                method_column = methods_list.index(method.getAttribute('method'))
                # fill the result list case by case
                result_list[method_column] = method.getAttribute('ok')
                # if even one method of the test cases failed remember the failure
                if not bool(int(method.getAttribute('ok'))):
                    _state.error_occurence_in_tests = True
                    # in case of an error we do not save '0' as result, but the error_description
                    result_list[method_column] = getText(method.childNodes)
            #===================================================================
            # after the results are now collected create the data row
            #===================================================================
            for cell_content in result_list:
                if cell_content == "-":
                    cell_color_attribute = "bgcolor=\"Yellow\""
                elif cell_content == "1":
                    cell_color_attribute = "bgcolor=\"Green\""
                else:
                    cell_color_attribute = "bgcolor=\"Red\""
                    cell_content = "<a title=\"" + cgi.escape(cell_content) + "\">0</a>"
                generate_html_table_cell(html, cell_content, cell_color_attribute)
            table_row_close()
    html = ["<html>"] + ["<body>"] + ["<table border=\"1\">"] + html + ["</table>"] + ["</body>"] + ["</html>"]
    return "\n".join(html)


#===============================================================================
# process command line arguments (i.e. file i/o)
#===============================================================================
if len(sys.argv) == 1:
    print("No command line arguments were given. Defaulting to:")
    print("Input '" + _config.input_file + "'")
    print("Output '" + _config.output_file + "'\n")
elif len(sys.argv) == 2:
    if sys.argv[1] == "--help":
        script_name = sys.argv[0][sys.argv[0].rfind("\\")+1:]
        print("Usage: " + script_name + " [INPUTFILE] [OUTPUTFILE]")
        sys.exit()
    else:
        _config.input_file = sys.argv[1]
        print("No special output was specified. Defaulting to:")
        print("Output '" + _config.output_file + "'\n")


#===============================================================================
# run the conversion
#===============================================================================
xmldom = xml.dom.minidom.parse(_config.input_file)
with open(_config.output_file, 'w') as html_file:
    html_file.write(convert_xml_2_html_table(xmldom.firstChild))


#===============================================================================
# concluding housekeeping 
#===============================================================================
print("Conversion done. See '" + _config.output_file + "'.")
if not _state.error_occurence_in_tests:
    print("All tests did succeed.")
else:
    print("Evaluation: at least one of the tests failed.")
sys.exit(_state.error_occurence_in_tests)

