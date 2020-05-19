import struct
from lxml import etree

def AddPkg(parent, short_name, inner_type="ELEMENTS"):
    outer = ap_gen = etree.SubElement(parent, 'AR-PACKAGE')
    etree.SubElement(outer, 'SHORT-NAME').text = short_name
    inner = etree.SubElement(outer, inner_type)
    return inner

def AddElem(parent, short_name, outer_type="OUTER", inner_type="INNER"):
    outer = ap_gen = etree.SubElement(parent, outer_type)
    etree.SubElement(outer, 'SHORT-NAME').text = short_name
    inner = etree.SubElement(outer, inner_type)
    return inner
    
def NamedSub(parent, elem_name, short_name):
    elem = etree.SubElement(parent, elem_name)
    etree.SubElement(elem, "SHORT-NAME").text = short_name
    return elem



def add_dict(root, dict_tree):
    if type(dict_tree) == dict:
        for k, v in dict_tree.items():
            if k.startswith('@'):
                root.set(k[1:], v)
            elif k == 'attrib':
                for a, b in v.items():
                    root.set(a, b)
            elif k in ('#text', 'text'):
                root.text = v
            elif type(v) == list:
                for a in v:
                    add_dict(etree.SubElement(root, k), a)
            else:
                add_dict(etree.SubElement(root, k), v)
    else:
        root.text = str(dict_tree)


gen_name = "PythonEtreeGenerated"
db_name = "Odrive_DB"
cluster_name = "OdriveCluster"
channel_name = "CANChannel"

def get_can():
    attr_qname = etree.QName("http://www.w3.org/2001/XMLSchema-instance", "schemaLocation")

    root = etree.Element('AUTOSAR',
                        {attr_qname: 'http://autosar.org/schema/r4.0 AUTOSAR_4-3-0.xsd'},
                        nsmap={None: 'http://autosar.org/schema/r4.0',
                                'xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                })
    ar_packages = etree.Element('AR-PACKAGES')
    root.append(ar_packages)
    ap_gen = AddPkg(ar_packages, gen_name, 'AR-PACKAGES')
    ap_sys = AddPkg(ap_gen, "SYSTEM")
    etree.SubElement(etree.SubElement(ap_sys, "SYSTEM"), "SHORT-NAME").text = "System"
    ap_cluster = AddPkg(ap_gen, db_name)
    cluster_var = AddElem(ap_cluster, cluster_name, "CAN-CLUSTER", "CAN-CLUSTER-VARIANTS")
    cluster_cond = etree.SubElement(cluster_var, "CAN-CLUSTER-CONDITIONAL")
    etree.SubElement(cluster_cond, "BAUDRATE").text = "250000"
    channels = etree.SubElement(cluster_cond, "PHYSICAL-CHANNELS")
    channel = etree.SubElement(channels, "CAN-PHYSICAL-CHANNEL")
    etree.SubElement(channel, "SHORT-NAME").text = channel_name
    comm_connectors = etree.SubElement(channel, "COMM-CONNECTORS") 
    triggerings = etree.SubElement(channel, "FRAME-TRIGGERINGS")
    sig_triggerings = etree.SubElement(channel, "I-SIGNAL-TRIGGERINGS")
    pdu_triggerings = etree.SubElement(channel, "PDU-TRIGGERINGS")
    etree.SubElement(cluster_cond, "PROTOCOL-NAME").text = "CAN"
    etree.SubElement(cluster_cond, "SPEED").text = "250000"
    ecu_instances = AddPkg(ap_gen, "ECU_INSTANCES")
    frm_elems = AddPkg(ap_gen, "FRAME")
    pdu_elems = AddPkg(ap_gen, "PDUS")
    sig_elems = AddPkg(ap_gen, "I_SIGNALS")
    types = AddPkg(ap_gen, "BASE_TYPES")
    add_dict(types, {"SW-BASE-TYPE": [
        {"SHORT-NAME":"UINT32", "BASE-TYPE-SIZE":"32", "BASE-TYPE-ENCODING":"NONE"},
        {"SHORT-NAME":"SINT32", "BASE-TYPE-SIZE":"32", "BASE-TYPE-ENCODING":"2C"},
        {"SHORT-NAME":"UINT16", "BASE-TYPE-SIZE":"16", "BASE-TYPE-ENCODING":"NONE"},
        {"SHORT-NAME":"SINT16", "BASE-TYPE-SIZE":"16", "BASE-TYPE-ENCODING":"2C"},
        {"SHORT-NAME":"UINT8",  "BASE-TYPE-SIZE":"8",  "BASE-TYPE-ENCODING":"NONE"},
        {"SHORT-NAME":"SINT8",  "BASE-TYPE-SIZE":"8",  "BASE-TYPE-ENCODING":"2C"},
        {"SHORT-NAME":"IEEE754","BASE-TYPE-SIZE":"32", "BASE-TYPE-ENCODING":"IEEE754"},
    ]})


    ap_compumethods = AddPkg(ap_gen, "COMPUMETHODS")
    add_dict(ap_compumethods, {"COMPU-METHOD": {
            "SHORT-NAME":{'text':"FACTOR_0_1"},
            "COMPU-INTERNAL-TO-PHYS": {
                "COMPU-SCALES": {
                    "COMPU-SCALE": {
                        "COMPU-RATIONAL-COEFFS": {
                            "COMPU-NUMERATOR": {"V": [0, 0.1]},
                            "COMPU-DENOMINATOR":{"V":1}
                        }
                    }
                }
            }
        }})
    add_dict(ap_compumethods, {"COMPU-METHOD": {
            "SHORT-NAME":{'text':"FACTOR_1"},
            "COMPU-INTERNAL-TO-PHYS": {
                "COMPU-SCALES": {
                    "COMPU-SCALE": {
                        "COMPU-RATIONAL-COEFFS": {
                            "COMPU-NUMERATOR": {"V": [0, 1]},
                            "COMPU-DENOMINATOR":{"V":1}
                        }
                    }
                }
            }
        }})
    add_dict(ap_compumethods, {"COMPU-METHOD": {
            "SHORT-NAME":{'text':"FACTOR_0_01"},
            "COMPU-INTERNAL-TO-PHYS": {
                "COMPU-SCALES": {
                    "COMPU-SCALE": {
                        "COMPU-RATIONAL-COEFFS": {
                            "COMPU-NUMERATOR": {"V": [0, 0.01]},
                            "COMPU-DENOMINATOR":{"V":1}
                        }
                    }
                }
            }
        }})


    syssig_elems = AddPkg(ap_gen, "SYSTEM_SIGNALS")
    return root


def add_ecu(root, ecu_name):
    comm_connectors = root.find(".//COMM-CONNECTORS")
    add_dict(comm_connectors, {"COMMUNICATION-CONNECTOR-REF-CONDITIONAL": {
        "COMMUNICATION-CONNECTOR-REF": {'@DEST':"CAN-COMMUNICATION-CONNECTOR", '#text': f"/{gen_name}/ECU_INSTANCES/{ecu_name}/Connector_{ecu_name}"}}}
    )
    ecu_instances = root.find(".//AR-PACKAGE[SHORT-NAME='ECU_INSTANCES']/ELEMENTS") 
    ecu = etree.SubElement(ecu_instances, "ECU-INSTANCE")
    etree.SubElement(ecu, "SHORT-NAME").text = ecu_name
    
    apdu_refs = etree.SubElement(ecu, "ASSOCIATED-COM-I-PDU-GROUP-REFS")
    # TODO PDU-GROUP
    
    ctrls = etree.SubElement(ecu, "COMM-CONTROLLERS")
    ctrl = etree.SubElement(ctrls, "CAN-COMMUNICATION-CONTROLLER")
    etree.SubElement(ctrl, "SHORT-NAME").text = f"Controller_{ecu_name}"
    varis = etree.SubElement(ctrl, "CAN-COMMUNICATION-CONTROLLER-VARIANTS")
    cond = etree.SubElement(varis, "CAN-COMMUNICATION-CONTROLLER-CONDITIONAL")
    attrs = etree.SubElement(cond, "CAN-CONTROLLER-ATTRIBUTES")
    conf = etree.SubElement(attrs, "CAN-CONTROLLER-CONFIGURATION")
    
    conns = etree.SubElement(ecu, "CONNECTORS")
    con = etree.SubElement(conns, "CAN-COMMUNICATION-CONNECTOR")
    etree.SubElement(con, "SHORT-NAME").text = f"Connector_{ecu_name}"
    etree.SubElement(con, "COMM-CONTROLLER-REF", DEST="CAN-COMMUNICATION-CONTROLLER").text = \
        f"/{gen_name}/ECU_INSTANCES/{ecu_name}/Controller_{ecu_name}"
    
    ports = etree.SubElement(con, "ECU-COMM-PORT-INSTANCES")


def add_frame(root, frm_name, can_id, dlc, ecu_tx, ecu_rx, pdu):
    frm_elems = root.find(".//AR-PACKAGE[SHORT-NAME='FRAME']/ELEMENTS") 
    add_dict(frm_elems, {"CAN-FRAME":{
        "SHORT-NAME":frm_name, 
        "FRAME-LENGTH": str(dlc),
        "PDU-TO-FRAME-MAPPINGS": {"PDU-TO-FRAME-MAPPING":{
            "SHORT-NAME": f"PduToFrameMapping_{frm_name}_{pdu}",
             "PACKING-BYTE-ORDER": "MOST-SIGNIFICANT-BYTE-LAST",
             "PDU-REF": {"@DEST":"I-SIGNAL-I-PDU", "#text": f"/{gen_name}/PDUS/{pdu}"},
             "START-POSITION": "0",
        }}
    }})

    pdu_elems = root.find(".//AR-PACKAGE[SHORT-NAME='PDUS']/ELEMENTS") 
    ipdu = NamedSub(pdu_elems, "I-SIGNAL-I-PDU", pdu)
    etree.SubElement(ipdu, "LENGTH").text = str(dlc)
    sig_maps = etree.SubElement(ipdu, "I-SIGNAL-TO-PDU-MAPPINGS")

    pdu_triggerings = root.find(".//CAN-PHYSICAL-CHANNEL/PDU-TRIGGERINGS")    
    trigger = etree.SubElement(pdu_triggerings, "PDU-TRIGGERING")
    etree.SubElement(trigger, "SHORT-NAME").text = f"PduTriggering_{pdu}"    
    port_refs = etree.SubElement(trigger, "I-PDU-PORT-REFS")
    etree.SubElement(port_refs, "I-PDU-PORT-REF", DEST="I-PDU-PORT").text = \
        f"/{gen_name}/ECU_INSTANCES/{ecu_tx}/Connector_{ecu_tx}/PduPort_{ecu_tx}_TX_{pdu}"
    etree.SubElement(port_refs, "I-PDU-PORT-REF", DEST="I-PDU-PORT").text = \
        f"/{gen_name}/ECU_INSTANCES/{ecu_rx}/Connector_{ecu_rx}/PduPort_{ecu_rx}_RX_{pdu}"    
    etree.SubElement(trigger, "I-PDU-REF", DEST="I-SIGNAL-I-PDU").text = f"/{gen_name}/PDUS/{pdu}"
    sig_trigs = etree.SubElement(trigger, "I-SIGNAL-TRIGGERINGS") 

    ports = root.find(f".//ECU-INSTANCE[SHORT-NAME='{ecu_tx}']/CONNECTORS/CAN-COMMUNICATION-CONNECTOR/ECU-COMM-PORT-INSTANCES")
    frm_port = NamedSub(ports, "FRAME-PORT", f"FramePort_{ecu_tx}_TX_{frm_name}")
    etree.SubElement(frm_port, "COMMUNICATION-DIRECTION").text = "OUT"
    pdu_port = NamedSub(ports, "I-PDU-PORT", f"PduPort_{ecu_tx}_TX_{pdu}")
    etree.SubElement(pdu_port, "COMMUNICATION-DIRECTION").text = "OUT"
    
    ports = root.find(f".//ECU-INSTANCE[SHORT-NAME='{ecu_rx}']/CONNECTORS/CAN-COMMUNICATION-CONNECTOR/ECU-COMM-PORT-INSTANCES")
    frm_port = NamedSub(ports, "FRAME-PORT", f"FramePort_{ecu_rx}_RX_{frm_name}")
    etree.SubElement(frm_port, "COMMUNICATION-DIRECTION").text = "IN"
    pdu_port = NamedSub(ports, "I-PDU-PORT", f"PduPort_{ecu_rx}_RX_{pdu}")
    etree.SubElement(pdu_port, "COMMUNICATION-DIRECTION").text = "IN"

    triggerings = root.find(".//FRAME-TRIGGERINGS")
    f = {
        'CAN-FRAME-TRIGGERING':
        {
            "SHORT-NAME":f"FrameTriggering_{frm_name}",
            "FRAME-PORT-REFS": {"FRAME-PORT-REF":
            [
                {'@DEST':"FRAME-PORT", '#text':f"/{gen_name}/ECU_INSTANCES/{ecu_tx}/Connector_{ecu_tx}/FramePort_{ecu_tx}_TX_{frm_name}" },
                {"@DEST":"FRAME-PORT", '#text':f"/{gen_name}/ECU_INSTANCES/{ecu_rx}/Connector_{ecu_rx}/FramePort_{ecu_rx}_RX_{frm_name}" }
            ]},
            "FRAME-REF":{'@DEST':"CAN-FRAME", '#text':f"/{gen_name}/FRAME/{frm_name}"},
            "PDU-TRIGGERINGS":{
                "PDU-TRIGGERING-REF-CONDITIONAL":{
                    "PDU-TRIGGERING-REF": {'@DEST':"PDU-TRIGGERING", '#text':f"/{gen_name}/{db_name}/{cluster_name}/{channel_name}/PduTriggering_{pdu}" }
                }
            },            
            "CAN-ADDRESSING-MODE":"STANDARD",
            "CAN-FRAME-TX-BEHAVIOR":"CAN-20",
            "IDENTIFIER":str(can_id)
        }
    }
    add_dict(triggerings, f)

def add_signal(root, sig_name, len, sig_type, compu_meth='FACTOR_1'):
    sig_elems = root.find(".//AR-PACKAGE[SHORT-NAME='I_SIGNALS']/ELEMENTS") 
    sig = NamedSub(sig_elems, "I-SIGNAL", sig_name)
    etree.SubElement(sig, "DATA-TYPE-POLICY").text = "OVERRIDE"
    etree.SubElement(sig, "LENGTH").text = str(len)
    rep = etree.SubElement(sig, "NETWORK-REPRESENTATION-PROPS")
    vari = etree.SubElement(rep, "SW-DATA-DEF-PROPS-VARIANTS")
    cond = etree.SubElement(vari, "SW-DATA-DEF-PROPS-CONDITIONAL")
    etree.SubElement(cond, "BASE-TYPE-REF", DEST="SW-BASE-TYPE").text = \
        f"/{gen_name}/BASE_TYPES/{sig_type}"
    etree.SubElement(sig, "SYSTEM-SIGNAL-REF", DEST="SYSTEM-SIGNAL").text = \
        f"/{gen_name}/SYSTEM_SIGNALS/{sig_name}"

    sig_triggerings = root.find(".//CAN-PHYSICAL-CHANNEL/I-SIGNAL-TRIGGERINGS")
    trigger = etree.SubElement(sig_triggerings, "I-SIGNAL-TRIGGERING")
    etree.SubElement(trigger, "SHORT-NAME").text = f"SignalTriggering_{sig_name}"
    refs = etree.SubElement(trigger, "I-SIGNAL-PORT-REFS")        
    etree.SubElement(trigger, "I-SIGNAL-REF", DEST="I-SIGNAL").text = \
            f"/{gen_name}/I_SIGNALS/{sig_name}"

    syssig_elems = root.find(".//AR-PACKAGE[SHORT-NAME='SYSTEM_SIGNALS']/ELEMENTS")
    add_dict(syssig_elems, {"SYSTEM-SIGNAL":{
        "SHORT-NAME":sig_name, 
        "PHYSICAL-PROPS": {
            "SW-DATA-DEF-PROPS-VARIANTS": {
                "SW-DATA-DEF-PROPS-CONDITIONAL": {
                    "COMPU-METHOD-REF": {"@DEST":"COMPU-METHOD", "#text": f"/{gen_name}/COMPUMETHODS/{compu_meth}"}
                }
            }
        }
        }})


def add_signal_mapping(root, pdu_name, sig_name, start_bit, ecu_tx, ecu_rx):
    sig_maps = root.find(f".//I-SIGNAL-I-PDU[SHORT-NAME='{pdu_name}']/I-SIGNAL-TO-PDU-MAPPINGS")         
    sig_map = NamedSub(sig_maps, "I-SIGNAL-TO-I-PDU-MAPPING", f"SignalPduMapping_{pdu_name}_{sig_name}")
    etree.SubElement(sig_map, "I-SIGNAL-REF", DEST="I-SIGNAL").text = \
        f"/{gen_name}/I_SIGNALS/{sig_name}"
    etree.SubElement(sig_map, "PACKING-BYTE-ORDER").text = "MOST-SIGNIFICANT-BYTE-LAST"
    etree.SubElement(sig_map, "START-POSITION").text = str(start_bit)
    
    sig_trigs = root.find(f".//PDU-TRIGGERING[SHORT-NAME='PduTriggering_{pdu_name}']/I-SIGNAL-TRIGGERINGS")
    refs = etree.SubElement(sig_trigs, "I-SIGNAL-TRIGGERING-REF-CONDITIONAL")
    etree.SubElement(refs, "I-SIGNAL-TRIGGERING-REF",DEST="I-SIGNAL-TRIGGERING").text = \
        f"/{gen_name}/{db_name}/{cluster_name}/{channel_name}/SignalTriggering_{sig_name}"


    refs2 = root.find(f".//I-SIGNAL-TRIGGERING[SHORT-NAME='SignalTriggering_{sig_name}']/I-SIGNAL-PORT-REFS")
    etree.SubElement(refs2, "I-SIGNAL-PORT-REF", DEST="I-SIGNAL-PORT").text = \
        f"/{gen_name}/ECU_INSTANCES/{ecu_tx}/Connector_{ecu_tx}/SignalPort_{ecu_tx}_TX_{sig_name}" 
    etree.SubElement(refs2, "I-SIGNAL-PORT-REF", DEST="I-SIGNAL-PORT").text = \
        f"/{gen_name}/ECU_INSTANCES/{ecu_rx}/Connector_{ecu_rx}/SignalPort_{ecu_rx}_RX_{sig_name}" 

    
    ports = root.find(f".//ECU-INSTANCE[SHORT-NAME='{ecu_tx}']/CONNECTORS/CAN-COMMUNICATION-CONNECTOR/ECU-COMM-PORT-INSTANCES") 
    sig_port = NamedSub(ports, "I-SIGNAL-PORT", f"SignalPort_{ecu_tx}_TX_{sig_name}")
    etree.SubElement(sig_port, "COMMUNICATION-DIRECTION").text = "OUT"

    ports = root.find(f".//ECU-INSTANCE[SHORT-NAME='{ecu_rx}']/CONNECTORS/CAN-COMMUNICATION-CONNECTOR/ECU-COMM-PORT-INSTANCES") 
    sig_port = NamedSub(ports, "I-SIGNAL-PORT", f"SignalPort_{ecu_rx}_RX_{sig_name}")
    etree.SubElement(sig_port, "COMMUNICATION-DIRECTION").text = "IN"




TYPES = {'f':'IEEE754', 'B':'UINT8', '?':'UINT8', 'b':'SINT8', 'H':'UINT16', 'h':'SINT16', 'I':'UINT32', 'i':'SINT32'}


def add_pydb_frame(root, ecu_name, gw_name, can_id, frm_name, direction, formats, signals):
    (ecu_rx, ecu_tx) = (gw_name, ecu_name) if direction == 'RX' else (ecu_name, gw_name)

    if frm_name is None:
        frm_name = ecu_tx + "_0x%x" % (can_id,)
    dlc = struct.calcsize(formats)
    pdu = frm_name
    add_frame(root, frm_name, can_id, dlc, ecu_tx, ecu_rx, pdu)

    startbit = 0
    for n in range(len(signals)):
        sig = signals[n]
        sig_name = sig.replace('.', '_').replace('/', '_')
        sig_type = TYPES[formats[n]]
        sig_len = struct.calcsize(formats[n]) * 8
        add_signal(root, sig_name, sig_len, sig_type)

        add_signal_mapping(root, pdu, sig_name, startbit, ecu_tx, ecu_rx)
        startbit += sig_len



