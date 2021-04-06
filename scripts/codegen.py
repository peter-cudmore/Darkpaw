

class LookupTable:
    def __init__(self, table_type, **kwargs):
        self.table_type = table_type
        self.tuples = [(t.__name__, n) for (n, t) in kwargs.items()]
        self.tables = {}

    @property
    def item_name(self):
        return f"{self.table_type}Item"

    def get_header(self):

        tuples = [f"    {t} {name};" for t, name in self.tuples]

        lines = [
            f"typedef struct " + "{\n",
            "\n".join(tuples),
            "\n} " + f"{self.item_name};\n",
            "\n"
        ]
        lines += [
            f"extern {self.item_name} {name}[];\n"
            for name in self.tables
        ]

        return "".join(lines)

    def get_table_implementation(self):
        lines = []
        for table_name, contents in self.tables.items():
            entries = [
                "     {" + ", ".join(str(v_i) for v_i in v) + "}"
                for v in sorted(contents, key=lambda x: x[0])
            ]
            lines += [
                f"{self.item_name} {table_name}[] = ",
                "{\n",
                ",\n".join(entries),
                "\n};\n"
            ]

        return "\n".join(lines)

    def add_entry(self, table_name, *args):
        if table_name not in self.tables:
            self.tables[table_name] = []

        self.tables[table_name].append(args)


class Function:
    def __init__(self, name, ret_type):
        self.arguments = []
        self.name = name
        self.ret_type = ret_type
        self.statements = []

    def get_definition(self):
        output = self._signature() + "{\n"
        output += "\n".join([f"    {s}" for s in self.statements])
        output += "}\n"
        return output

    def _signature(self):
        output = f"{self.ret_type} {self.name}("
        output += ", ".join(f'{t} {n}' for t, n in self.arguments)
        output += ")"
        return output

    def get_declaration(self):
        return self._signature() + ";"



if __name__ == "__main__":
    test_table = LookupTable(
        'Test',
        pwm=int,
        value=float
    )

    test_table.add_entry(1, 2)
    test_table.add_entry(3, 4)
