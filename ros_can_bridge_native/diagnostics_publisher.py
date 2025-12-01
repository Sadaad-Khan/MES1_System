#!/usr/bin/env python3
"""
Diagnostics Publisher.

Publishes system diagnostics and health information.
"""

import time
from typing import Dict, Any

# Note: In production, import rclpy and ROS 2 messages
# For now, create a standalone version that will be integrated later


class DiagnosticsPublisher:
    """
    Publishes diagnostic information for system monitoring.
    
    Implements REP-107 (diagnostic_msgs) standard.
    """
    
    def __init__(self):
        """Initialize diagnostics publisher."""
        self.update_rate = 1.0  # Hz
        
        # Thresholds
        self.bus_load_warning = 0.7
        self.bus_load_error = 0.9
        self.error_rate_warning = 0.01
        self.error_rate_error = 0.05
        self.latency_warning = 10.0  # ms
        self.latency_error = 50.0  # ms
    
    def collect_diagnostics(self) -> Dict[str, Any]:
        """
        Collect diagnostic information.
        
        Returns:
            Dictionary of diagnostic data
        """
        # In production, query actual system state
        # For now, return simulated data
        
        diagnostics = {
            'can_bus': {
                'level': 'OK',  # OK, WARN, ERROR, STALE
                'message': 'CAN bus operational',
                'hardware_id': 'can0',
                'values': {
                    'interface': 'can0',
                    'state': 'UP',
                    'bitrate': 1000000,
                    'bus_load': 0.15,  # 15%
                    'frames_tx': 1234,
                    'frames_rx': 5678,
                    'errors_tx': 0,
                    'errors_rx': 0,
                    'error_rate': 0.0,
                }
            },
            'sdo_transactions': {
                'level': 'OK',
                'message': 'SDO transactions healthy',
                'values': {
                    'success_count': 145,
                    'failure_count': 2,
                    'success_rate': 0.986,
                    'avg_latency_ms': 3.5,
                    'max_latency_ms': 12.3,
                }
            },
            'node_health': {
                'level': 'OK',
                'message': 'Node operational',
                'values': {
                    'uptime_seconds': 3600,
                    'rx_thread_alive': True,
                    'tx_thread_alive': True,
                    'queue_utilization': 0.05,
                }
            }
        }
        
        # Apply thresholds
        bus_load = diagnostics['can_bus']['values']['bus_load']
        if bus_load > self.bus_load_error:
            diagnostics['can_bus']['level'] = 'ERROR'
            diagnostics['can_bus']['message'] = f'Bus load critical: {bus_load*100:.1f}%'
        elif bus_load > self.bus_load_warning:
            diagnostics['can_bus']['level'] = 'WARN'
            diagnostics['can_bus']['message'] = f'Bus load high: {bus_load*100:.1f}%'
        
        error_rate = diagnostics['can_bus']['values']['error_rate']
        if error_rate > self.error_rate_error:
            diagnostics['can_bus']['level'] = 'ERROR'
            diagnostics['can_bus']['message'] = f'Error rate critical: {error_rate*100:.1f}%'
        elif error_rate > self.error_rate_warning:
            diagnostics['can_bus']['level'] = 'WARN'
            diagnostics['can_bus']['message'] = f'Error rate elevated: {error_rate*100:.1f}%'
        
        return diagnostics
    
    def format_diagnostics(self, diagnostics: Dict[str, Any]) -> str:
        """
        Format diagnostics for console display.
        
        Args:
            diagnostics: Diagnostic data
        
        Returns:
            Formatted string
        """
        output = []
        output.append('=' * 70)
        output.append('System Diagnostics')
        output.append('=' * 70)
        
        for component, data in diagnostics.items():
            level_indicator = {
                'OK': '✓',
                'WARN': '⚠',
                'ERROR': '✗',
                'STALE': '?'
            }.get(data['level'], '?')
            
            output.append(f'\n[{level_indicator}] {component.replace("_", " ").title()}')
            output.append(f'    Status: {data["level"]} - {data["message"]}')
            
            if 'hardware_id' in data:
                output.append(f'    Hardware: {data["hardware_id"]}')
            
            if 'values' in data:
                output.append('    Metrics:')
                for key, value in data['values'].items():
                    if isinstance(value, float):
                        output.append(f'      {key}: {value:.3f}')
                    else:
                        output.append(f'      {key}: {value}')
        
        output.append('=' * 70)
        
        return '\n'.join(output)
    
    def run(self):
        """Run diagnostics publisher (standalone mode)."""
        print('Diagnostics Publisher')
        print('Press Ctrl+C to exit')
        print()
        
        try:
            while True:
                diagnostics = self.collect_diagnostics()
                output = self.format_diagnostics(diagnostics)
                
                # Clear screen and print
                print('\033[2J\033[H')  # ANSI clear screen
                print(output)
                
                time.sleep(1.0 / self.update_rate)
        
        except KeyboardInterrupt:
            print('\nDiagnostics publisher stopped.')


def main(args=None):
    """Main entry point."""
    # Note: In production, initialize as ROS 2 node
    # import rclpy
    # rclpy.init(args=args)
    # node = DiagnosticsPublisherNode()
    # rclpy.spin(node)
    # rclpy.shutdown()
    
    publisher = DiagnosticsPublisher()
    publisher.run()


if __name__ == '__main__':
    main()
