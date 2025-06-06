class RiskAnalyzer:
    """Analyzes risk levels and emits alerts"""
    
    def __init__(self, node):
        self.node = node
        
    def emit_alert(self, patient_status):
        """Emit alerts based on patient status and sensor risk levels"""
        # Skip if not active
        if not self.node.active or patient_status <= 0:
            return
            
        self.node.publisher_manager.publish_status("activated", "emit_emergency")
        
        # Categorize patient status
        if patient_status <= 20.0:
            risk_category = "VERY LOW RISK"
        elif 20.0 < patient_status <= 40.0:
            risk_category = "LOW RISK"
        elif 40.0 < patient_status <= 60.0:
            risk_category = "MODERATE RISK"
        elif 60.0 < patient_status <= 80.0:
            risk_category = "CRITICAL RISK"
            self.node.get_logger().fatal(
                f"[Emergency Detection]\n SYSTEM ALERT: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        elif 80.0 < patient_status <= 100.0:
            risk_category = "VERY CRITICAL RISK"
            self.node.get_logger().fatal(
                f"[Emergency Detection]\n SYSTEM ALERT: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        else:
            risk_category = "UNKNOWN RISK"
        
        # Log moderate/low risk (critical alerts already handled above)
        if 40.0 < patient_status <= 60.0:
            self.node.get_logger().warning(
                f"[Emergency Detection]\n SYSTEM WARNING: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        elif patient_status <= 40.0 and patient_status > 0:
            self.node.get_logger().info(
                f"[Emergency Detection]\n System Status: {risk_category} - {patient_status:.1f}%\n"
            )
            
        self.node.publisher_manager.publish_status("activated", "idle")