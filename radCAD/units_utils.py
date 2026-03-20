# units_utils.py
# The Math Brain: Handles Imperial/Metric conversions and parsing

import bpy
from .modal_state import state

def format_fraction_part(inches_val):
    """Helper to calculate whole inches and fractional numerator (1/16ths)."""
    whole_inches = int(inches_val)
    fraction_part = inches_val - whole_inches
    numerator = int(round(fraction_part * 16))
    
    if numerator == 16:
        whole_inches += 1
        numerator = 0
        
    return whole_inches, numerator

def format_length(val_meters):
    """
    Formats meters into a string based on the scene's EXACT unit context.
    """
    units = bpy.context.scene.unit_settings
    system = units.system
    l_unit = units.length_unit
    
    if system == 'IMPERIAL':
        total_inches = val_meters * 39.37007874
        snapped_inches = round(total_inches * 16) / 16
        
        is_dirty = abs(total_inches - snapped_inches) > 0.001
        prefix = "~ " if is_dirty else ""
        
        if l_unit == 'INCHES':
            whole_i, num = format_fraction_part(snapped_inches)
            den = 16
            if num > 0:
                while num % 2 == 0 and den % 2 == 0:
                    num //= 2
                    den //= 2
            s = f"{whole_i}"
            if num > 0: s += f" {num}/{den}"
            return f'{prefix}{s}"'
        else: 
            feet = int(snapped_inches // 12)
            inches_dec = snapped_inches % 12
            whole_i, num = format_fraction_part(inches_dec)
            if whole_i == 12:
                feet += 1
                whole_i = 0
            den = 16
            if num > 0:
                while num % 2 == 0 and den % 2 == 0:
                    num //= 2
                    den //= 2
            parts = []
            if feet > 0: parts.append(f"{feet}'")
            inch_part = ""
            if whole_i > 0: inch_part = f"{whole_i}"
            elif feet == 0 and num == 0: inch_part = "0"
            if num > 0:
                if inch_part: inch_part += " "
                inch_part += f"{num}/{den}"
            if inch_part: parts.append(f'{inch_part}"')
            return f"{prefix}{' '.join(parts)}"

    # METRIC / GENERIC
    precision = state.get("display_precision", 3) 
    conversions = {
        'METERS': (1.0, "m"), 'KILOMETERS': (0.001, "km"),
        'CENTIMETERS': (100.0, "cm"), 'MILLIMETERS': (1000.0, "mm"),
        'MICROMETERS': (1000000.0, "µm"),
    }
    if l_unit in conversions:
        factor, suffix = conversions[l_unit]
        val = val_meters * factor
        if precision == 0 or abs(val - round(val)) < 1e-9:
             return f"{int(round(val))}{suffix}"
        return f"{val:.{precision}f}{suffix}"
    return bpy.utils.units.to_string(system, 'LENGTH', val_meters, precision=precision, split_unit=True)


def safe_eval_token(token):
    """Parses a single number or fraction (e.g. '5' or '5/8')."""
    try:
        if '/' in token:
            n, d = map(float, token.split('/', 1))
            return n / d if d != 0 else 0.0
        return float(token)
    except ValueError:
        return 0.0

def safe_eval_additive_string(val_str):
    """Parses '5 1/2' -> 5.5"""
    val_str = val_str.replace('-', ' ').strip()
    return sum(safe_eval_token(p) for p in val_str.split() if p.strip())

def parse_imperial_string(input_string):
    """Parses explicit imperial strings with quotes."""
    s = input_string.replace('"', '').replace("''", "") 
    total_inches = 0.0
    
    if "'" in input_string:
        parts = s.split("'")
        if parts[0].strip():
            total_inches += safe_eval_additive_string(parts[0]) * 12.0
        if len(parts) > 1 and parts[1].strip():
            total_inches += safe_eval_additive_string(parts[1])
    else:
        total_inches = safe_eval_additive_string(s)

    return total_inches * 0.0254

def parse_implicit_imperial(val_str):
    """
    Handles lazy syntax '5 6' or '5 5 5/8' -> Feet + Inches.
    Returns None if the string contains non-numeric characters (like 'm', 'cm').
    """
    # 1. Clean formatting
    val_str = val_str.replace('-', ' ').strip()
    parts = val_str.split()
    
    # 2. Safety Check: If any part has letters, it's probably Metric (e.g. '1 m')
    for p in parts:
        if any(c.isalpha() for c in p):
            return None
            
    # 3. Must be multi-part to assume Feet + Inches
    if len(parts) < 2:
        return None
        
    # 4. Logic: First Token = FEET, Rest = INCHES
    try:
        feet = safe_eval_token(parts[0])
        inches = sum(safe_eval_token(p) for p in parts[1:])
        return (feet * 12.0 + inches) * 0.0254
    except:
        return None

def parse_length_input(val_str):
    # 1. Explicit Quotes -> Imperial
    if "'" in val_str or '"' in val_str:
        return parse_imperial_string(val_str)

    # 2. Lazy Contractor Mode (Implicit Feet + Inches)
    # Works if we see spaces and strictly numeric tokens
    implicit_val = parse_implicit_imperial(val_str)
    if implicit_val is not None:
        return implicit_val

    # 3. System Fallback (Metric/Imperial standard)
    units = bpy.context.scene.unit_settings
    system = units.system
    
    try:
        # If raw number in Metric, append default unit
        if system == 'METRIC':
             try: 
                 float(val_str)
                 suffix = {"MILLIMETERS": "mm", "CENTIMETERS": "cm", "METERS": "m", "KILOMETERS": "km"}.get(units.length_unit, "m")
                 val_str = f"{val_str}{suffix}"
             except: pass
             
        return bpy.utils.units.to_value(system, 'LENGTH', val_str)
    except:
        return 0.0