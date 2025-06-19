import can
import time
import datetime
from tkinter import *
from tkinter import ttk
import threading
from queue import Queue

class CANDataLogger:
    def __init__(self):
        # Initialize CAN bus
        self.bus = self.connect_to_can()
        self.running = True
        
        # Create log file
        self.presentDate = datetime.datetime.now()
        self.log_name = input("Enter log name: ")
        self.name = f"{self.presentDate.date()}_{self.presentDate.hour}_{self.presentDate.minute}_{self.presentDate.second}_{self.log_name}"
        self.path_name = "D:\\ChargerLog\\"
        self.log_file_name = f"{self.path_name}{self.name}.txt"
        print(f"Logging to: {self.log_file_name}")
        
        # Field names
        self.field_names = ['time_stamp','Output_voltage','Output_current','Status_flag','Charger_internal_temperature','AC_input_voltage','AC_input_current',
                          'Power_factor','Active_power','Reactive_power','Instantaneous_efficiency']
        
        # Data storage
        self.current_data = {}
        self.previous_data = {}  # To track previous values for highlighting
        self.flag = False
        
        # GUI update queue
        self.gui_queue = Queue()
        
        # Start threads
        self.can_thread = threading.Thread(target=self.read_can_data)
        self.can_thread.daemon = True
        self.can_thread.start()
        
        # Initialize GUI
        self.root = Tk()
        self.root.title("Real-time CAN Data Logger")
        self.setup_gui()
        
        # Start GUI update loop
        self.update_gui()
        self.root.mainloop()
    
    def connect_to_can(self):
        try:
            can.rc['interface'] = 'pcan'
            can.rc['bitrate'] = 250000
            return can.interface.Bus()
        except Exception as e:
            print(f"CAN Connection Error: {e}")
            return None
    
    def create_format(self, data_dict):
        formatted = {}
        formatted['time_stamp'] = data_dict['time_stamp']
        formatted['output_voltage'] = data_dict['Output_voltage']
        formatted['output_current'] = data_dict['Output_current']
        formatted['status_flag'] = data_dict['Status_flag']
        formatted['charger_internal_temperature'] = data_dict['Charger_internal_temperature']
        formatted['input_voltage'] = data_dict['AC_input_voltage']
        formatted['input_current'] = data_dict['AC_input_current']
        formatted['power_factor'] = data_dict['Power_factor']
        formatted['active_power'] = data_dict['Active_power']
        formatted['reactive_power'] = data_dict['Reactive_power']
        formatted['instantaneous_effieciency'] = data_dict['Instantaneous_efficiency']
        return formatted
    
    def read_can_data(self):
        while self.running and self.bus:
            try:
                dict = {}
                can_msg = str(self.bus.recv()) + "\n"
                
                for msg in can_msg:
                    line = str(self.bus.recv()) + "\n"
                    x = line.split()
                    if len(x) < 5:
                        continue
                    
                    del x[0]
                    del x[4]
                    
                    dict['time_stamp'] = str(datetime.datetime.now())
                    
                    if x[2] == '18ff50e5':         
                        O_v= ((int(x[7] + x[6], 16)))
                        dict['Output_voltage']= O_v/10
                        O_c= ((int(x[9] + x[8], 16)))
                        dict['Output_current']=O_c/10
                        S_f=int(x[10],16)
                        dict['Status_flag'] =S_f
                        C_I_T=int(x[11],16)
                        dict['Charger_internal_temperature']=C_I_T
                        AC_iv=int(x[12],16)
                        dict['AC_input_voltage']=AC_iv
                        AC_ic=int(x[13],16)
                        dict['AC_input_current']=AC_ic
                        self.flag = True

                    if x[2] == '18ff50e6' and self.flag:
                        P_f=int(x[6],16)
                        dict['Power_factor'] = P_f
                        A_p=int(x[7]+x[8],16)
                        dict['Active_power'] = A_p/10
                        R_p=int(x[10]+x[9], 16)
                        dict['Reactive_power'] = R_p/10
                        I_e=int(x[11],16)
                        dict['Instantaneous_efficiency'] = I_e/100

                    if len(dict) >= len(self.field_names):
                    # if len(dict) >= 2:
                        try:
                            dict_new = self.create_format(dict)
                            
                            # Store current data for comparison
                            self.current_data = dict_new.copy()
                            
                            # Write to file
                            with open(self.log_file_name, 'a') as txt_file:
                                txt_file.write(str(dict_new) + '\n')
                            
                            # Send to GUI
                            self.gui_queue.put(dict_new.copy())
                            
                        except Exception as e:
                            print(f"Error processing data: {e}")
                        
                        dict = {}
                        self.flag = False
                        
            except Exception as e:
                print(f"Error in CAN reading: {e}")
                time.sleep(0.1)
    
    def setup_gui(self):
        # Create main frame
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(N, S, E, W))
        
        # Create treeview with custom style for changing values
        self.style = ttk.Style()
        self.style.configure("Treeview", font=('Arial', 10))
        self.style.configure("Treeview.Heading", font=('Arial', 10, 'bold'))
        self.style.map('Treeview', background=[('selected', 'light blue')])
        
        # Define a new style for changed values
        self.style.configure("Changed.Treeview", background="light green")
        
        self.tree = ttk.Treeview(self.main_frame, columns=('Parameter', 'Value'), show='headings')
        self.tree.heading('Parameter', text='Parameter')
        self.tree.heading('Value', text='Value')
        self.tree.column('Parameter', width=150)
        self.tree.column('Value', width=150)
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(self.main_frame, orient=VERTICAL, command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        
        # Grid layout
        self.tree.grid(row=0, column=0, sticky=(N, S, E, W))
        scrollbar.grid(row=0, column=1, sticky=(N, S))
        
        # Configure resizing
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.rowconfigure(0, weight=1)
        
        # Close handler
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def update_gui(self):
        try:
            # Process all pending GUI updates
            while not self.gui_queue.empty():
                data = self.gui_queue.get_nowait()
                
                # Clear previous data
                for item in self.tree.get_children():
                    self.tree.delete(item)
                
                # Add new data and highlight changed values
                for key, value in data.items():
                    item_id = self.tree.insert('', 'end', values=(key, value))
                    
                    # Check if value has changed from previous data
                    if key in self.previous_data and self.previous_data[key] != value:
                        self.tree.item(item_id, tags=('changed',))
                        self.tree.tag_configure('changed', background='light green')
                
                # Store current data as previous for next comparison
                self.previous_data = data.copy()
        
        except Exception as e:
            print(f"GUI update error: {e}")
        
        # Schedule next update
        self.root.after(100, self.update_gui)
    
    def on_close(self):
        self.running = False
        if self.bus:
            self.bus.shutdown()
        self.root.destroy()

if __name__ == "__main__":
    logger = CANDataLogger()