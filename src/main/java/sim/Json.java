package sim;

import java.util.List;

/**
 * Tiny JSON writer so the sim doesn't need a library.
 */
public final class Json {

    private final StringBuilder sb = new StringBuilder();
    private boolean needComma = false;

    private void comma() {
        if (needComma) sb.append(',');
        needComma = false;
    }

    public Json begin() {
        comma();
        sb.append('{');
        return this;
    }

    public Json end() {
        sb.append('}');
        needComma = true;
        return this;
    }

    public Json beginArray() {
        comma();
        sb.append('[');
        return this;
    }

    public Json endArray() {
        sb.append(']');
        needComma = true;
        return this;
    }

    public Json key(String key) {
        comma();
        quote(key);
        sb.append(':');
        return this;
    }

    public Json field(String key, String value) {
        key(key);
        if (value == null) sb.append("null");
        else quote(value);
        needComma = true;
        return this;
    }

    public Json field(String key, double value) {
        key(key);
        sb.append(Double.isFinite(value) ? String.format(java.util.Locale.ROOT, "%.4f", value) : "null");
        needComma = true;
        return this;
    }

    public Json field(String key, boolean value) {
        key(key);
        sb.append(value);
        needComma = true;
        return this;
    }

    public Json numberArray(String key, double[] values) {
        key(key);
        sb.append('[');
        for (int i = 0; i < values.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(String.format(java.util.Locale.ROOT, "%.1f", values[i]));
        }
        sb.append(']');
        needComma = true;
        return this;
    }

    public Json stringArray(String key, List<String> values) {
        key(key);
        sb.append('[');
        for (int i = 0; i < values.size(); i++) {
            if (i > 0) sb.append(',');
            quote(values.get(i));
        }
        sb.append(']');
        needComma = true;
        return this;
    }

    private void quote(String s) {
        sb.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"': sb.append("\\\""); break;
                case '\\': sb.append("\\\\"); break;
                case '\n': sb.append("\\n"); break;
                case '\r': sb.append("\\r"); break;
                case '\t': sb.append("\\t"); break;
                default:
                    if (c < 0x20) sb.append(String.format("\\u%04x", (int) c));
                    else sb.append(c);
            }
        }
        sb.append('"');
    }

    @Override
    public String toString() {
        return sb.toString();
    }
}
